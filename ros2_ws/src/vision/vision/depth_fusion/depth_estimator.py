"""Depth fusion utilities for combining 2D detections with dense depth images."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, MutableMapping, Optional, Sequence, Tuple

import numpy as np


@dataclass
class DepthFusionClassConfig:
    """Parameter bundle for a single class."""

    sampling_stride: int = 4
    min_samples: int = 30
    z_min: float = 0.2
    z_max: float = 12.0
    region_type: str = "center"
    band_ratio: float = 0.4
    gate_k_iqr: float = 1.5
    gate_percentile: float = 0.75
    cluster_eps: float = 0.25
    cluster_min_samples: int = 15
    prefer_closer_cluster: bool = True
    ransac_iters: int = 30
    ransac_tau: float = 0.1
    ransac_min_inliers: int = 8
    min_inlier_ratio: float = 0.2

    def update_from(self, overrides: Mapping[str, Any]) -> None:
        for key, value in overrides.items():
            attr = key
            if hasattr(self, attr):
                setattr(self, attr, type(getattr(self, attr))(value))


@dataclass
class BoundingBox:
    center_x: float
    center_y: float
    width: float
    height: float


_DEFAULT_CONFIG = DepthFusionClassConfig()
_CLASS_CONFIGS: Dict[str, DepthFusionClassConfig] = {"__default__": _DEFAULT_CONFIG}


def configure_depth_fusion(config_map: Mapping[str, Any]) -> None:
    """Configure depth fusion parameters from a nested mapping.

    Expected schema:
    {
        "defaults": {
            "sampling": {"stride": int, "min_samples": int},
            "valid_range": {"z_min": float, "z_max": float},
            "region": {"region_type": str, "band_ratio": float},
            "gate": {"k_iqr": float, "q_percentile": float},
            "cluster": {"eps_z": float, "min_samples": int},
            "ransac": {"iters": int, "tau_z": float, "min_inliers": int, "min_inlier_ratio": float}
        },
        "classes": {
            "gate": {
                ... (same keys as defaults but optional)
            },
            ...
        }
    }
    """

    global _CLASS_CONFIGS

    if not config_map:
        _CLASS_CONFIGS = {"__default__": DepthFusionClassConfig()}
        return

    defaults = _parse_class_config(config_map.get("defaults", {}), DepthFusionClassConfig())
    class_configs: Dict[str, DepthFusionClassConfig] = {"__default__": defaults}
    classes = config_map.get("classes", {})
    if isinstance(classes, Mapping):
        for class_name, override in classes.items():
            if not isinstance(override, Mapping):
                continue
            cfg = _parse_class_config(override, DepthFusionClassConfig(**defaults.__dict__))
            class_configs[str(class_name)] = cfg
    _CLASS_CONFIGS = class_configs


def estimate_depth_for_detection(
    bbox: Any,
    class_id: Any,
    depth_image: np.ndarray,
) -> Tuple[float, float, Dict[str, Any], bool]:
    """Estimate object depth inside bbox using robust filtering.

    Args:
        bbox: bounding box expressed as dataclass, tuple (cx, cy, w, h),
              or mapping with keys {cx, cy, w, h} / {center_x, ...}.
        class_id: class label string or index; coerced to str for config lookup.
        depth_image: numpy array of depth values (meters).

    Returns:
        depth_est: median depth of inliers (float or NaN)
        confidence: ratio of inliers vs. valid samples
        debug_info: dictionary with stats and baseline depth
        valid_flag: True if RANSAC produced a reliable estimate
    """

    debug: Dict[str, Any] = {}
    bbox_obj = _normalize_bbox(bbox)
    if bbox_obj is None or depth_image is None:
        debug["failure_reason"] = "invalid_bbox_or_image"
        return float("nan"), 0.0, debug, False

    if depth_image.ndim != 2:
        debug["failure_reason"] = "depth_not_2d"
        return float("nan"), 0.0, debug, False

    cfg = _CLASS_CONFIGS.get(str(class_id), _CLASS_CONFIGS.get("__default__", _DEFAULT_CONFIG))
    debug.update({
        "class_id": str(class_id),
        "region_type": cfg.region_type,
    })

    roi, pixel_bounds = _extract_roi(depth_image, bbox_obj)
    if roi.size == 0:
        debug["failure_reason"] = "empty_roi"
        return float("nan"), 0.0, debug, False
    debug["bbox_pixels"] = pixel_bounds

    baseline_depth = _compute_baseline_depth(roi, cfg)
    debug["baseline_depth"] = baseline_depth

    stride = max(1, int(cfg.sampling_stride))
    sampled = roi[::stride, ::stride]
    sample_h, sample_w = sampled.shape
    debug["stride"] = stride
    debug["sample_grid"] = (int(sample_w), int(sample_h))

    region_mask = _build_region_mask(sample_h, sample_w, cfg.region_type, cfg.band_ratio)
    debug["region_ratio"] = float(cfg.band_ratio)

    region_samples = sampled[region_mask]
    debug["region_sample_count"] = int(region_mask.sum())

    valid_mask = np.isfinite(region_samples)
    if cfg.z_min is not None:
        valid_mask &= region_samples >= cfg.z_min
    if cfg.z_max is not None:
        valid_mask &= region_samples <= cfg.z_max

    valid_depths = region_samples[valid_mask]
    debug["valid_samples"] = int(valid_depths.size)

    if valid_depths.size < max(1, int(cfg.min_samples)):
        debug["failure_reason"] = "insufficient_valid_samples"
        return float("nan"), 0.0, debug, False

    gate_mask, thresh_info = _apply_gate(valid_depths, cfg)
    gated_depths = valid_depths[gate_mask]
    debug.update(thresh_info)
    debug["gated_samples"] = int(gated_depths.size)

    if gated_depths.size < cfg.cluster_min_samples:
        debug["failure_reason"] = "insufficient_gated_samples"
        return float("nan"), 0.0, debug, False

    clusters = _cluster_depths(gated_depths, eps=cfg.cluster_eps, min_samples=cfg.cluster_min_samples)
    debug["cluster_candidates"] = len(clusters)

    if not clusters:
        debug["failure_reason"] = "no_density_cluster"
        return float("nan"), 0.0, debug, False

    chosen_cluster = _select_cluster(clusters, prefer_closer=cfg.prefer_closer_cluster)
    debug["cluster_size"] = chosen_cluster.size

    inliers = _run_constant_ransac(
        chosen_cluster,
        iters=cfg.ransac_iters,
        tau=cfg.ransac_tau,
        min_inliers=cfg.ransac_min_inliers,
    )
    debug["ransac_candidates"] = int(chosen_cluster.size)
    debug["ransac_inliers"] = int(inliers.size)

    total_valid = max(1, float(valid_depths.size))

    if inliers.size < cfg.ransac_min_inliers or (inliers.size / total_valid) < cfg.min_inlier_ratio:
        debug["failure_reason"] = "ransac_failed"
        return float("nan"), inliers.size / total_valid, debug, False

    depth_est = float(np.median(inliers))
    refined_inliers = chosen_cluster[np.abs(chosen_cluster - depth_est) <= cfg.ransac_tau]
    if refined_inliers.size:
        inliers = refined_inliers
        depth_est = float(np.median(refined_inliers))
    confidence = float(inliers.size / total_valid)

    debug["final_inliers"] = int(inliers.size)
    debug["depth_estimate"] = depth_est

    return depth_est, confidence, debug, True


def _parse_class_config(source: Mapping[str, Any], base: DepthFusionClassConfig) -> DepthFusionClassConfig:
    cfg = DepthFusionClassConfig(**base.__dict__)
    sampling = source.get("sampling", {}) if isinstance(source, Mapping) else {}
    if isinstance(sampling, Mapping):
        if "stride" in sampling:
            cfg.sampling_stride = int(sampling["stride"])
        if "min_samples" in sampling:
            cfg.min_samples = int(sampling["min_samples"])
    valid_range = source.get("valid_range", {}) if isinstance(source, Mapping) else {}
    if isinstance(valid_range, Mapping):
        if "z_min" in valid_range:
            cfg.z_min = float(valid_range["z_min"])
        if "z_max" in valid_range:
            cfg.z_max = float(valid_range["z_max"])
    region = source.get("region", {}) if isinstance(source, Mapping) else {}
    if isinstance(region, Mapping):
        if "region_type" in region:
            cfg.region_type = str(region["region_type"])
        if "band_ratio" in region:
            cfg.band_ratio = float(region["band_ratio"])
    gate = source.get("gate", {}) if isinstance(source, Mapping) else {}
    if isinstance(gate, Mapping):
        if "k_iqr" in gate:
            cfg.gate_k_iqr = float(gate["k_iqr"])
        if "q_percentile" in gate:
            cfg.gate_percentile = float(gate["q_percentile"])
    cluster = source.get("cluster", {}) if isinstance(source, Mapping) else {}
    if isinstance(cluster, Mapping):
        if "eps_z" in cluster:
            cfg.cluster_eps = float(cluster["eps_z"])
        if "min_samples" in cluster:
            cfg.cluster_min_samples = int(cluster["min_samples"])
        if "prefer_closer" in cluster:
            cfg.prefer_closer_cluster = bool(cluster["prefer_closer"])
    ransac = source.get("ransac", {}) if isinstance(source, Mapping) else {}
    if isinstance(ransac, Mapping):
        if "iters" in ransac:
            cfg.ransac_iters = int(ransac["iters"])
        if "tau_z" in ransac:
            cfg.ransac_tau = float(ransac["tau_z"])
        if "min_inliers" in ransac:
            cfg.ransac_min_inliers = int(ransac["min_inliers"])
        if "min_inlier_ratio" in ransac:
            cfg.min_inlier_ratio = float(ransac["min_inlier_ratio"])
    # Allow flat overrides at top level as fallbacks
    flat_keys = {
        "sampling_stride": "sampling_stride",
        "min_samples": "min_samples",
        "z_min": "z_min",
        "z_max": "z_max",
        "region_type": "region_type",
        "band_ratio": "band_ratio",
        "gate_k_iqr": "gate_k_iqr",
        "gate_percentile": "gate_percentile",
        "cluster_eps": "cluster_eps",
        "cluster_min_samples": "cluster_min_samples",
        "prefer_closer_cluster": "prefer_closer_cluster",
        "ransac_iters": "ransac_iters",
        "ransac_tau": "ransac_tau",
        "ransac_min_inliers": "ransac_min_inliers",
        "min_inlier_ratio": "min_inlier_ratio",
    }
    for key, attr in flat_keys.items():
        if key in source:
            setattr(cfg, attr, type(getattr(cfg, attr))(source[key]))
    return cfg


def _normalize_bbox(bbox: Any) -> Optional[BoundingBox]:
    if bbox is None:
        return None
    if isinstance(bbox, BoundingBox):
        return bbox
    if isinstance(bbox, Mapping):
        cx = _first_available(bbox, ["cx", "center_x", "center_x_px", "x"])
        cy = _first_available(bbox, ["cy", "center_y", "center_y_px", "y"])
        w = _first_available(bbox, ["w", "width", "size_x"])
        h = _first_available(bbox, ["h", "height", "size_y"])
        if None in (cx, cy, w, h):
            return None
        return BoundingBox(float(cx), float(cy), float(w), float(h))
    if isinstance(bbox, Sequence) and len(bbox) == 4:
        return BoundingBox(float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
    for attr_set in (("center", "size"),):
        if hasattr(bbox, attr_set[0]) and hasattr(bbox, attr_set[1]):
            center = getattr(bbox, attr_set[0])
            size = getattr(bbox, attr_set[1])
            if hasattr(center, "x") and hasattr(center, "y") and hasattr(size, "x") and hasattr(size, "y"):
                return BoundingBox(float(center.x), float(center.y), float(size.x), float(size.y))
    for attr in ("bbox", "bounding_box"):
        if hasattr(bbox, attr):
            return _normalize_bbox(getattr(bbox, attr))
    return None


def _first_available(data: Mapping[str, Any], keys: Sequence[str]) -> Optional[float]:
    for key in keys:
        if key in data:
            return data[key]
    return None


def _extract_roi(depth_img: np.ndarray, bbox: BoundingBox) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
    height, width = depth_img.shape
    half_w = bbox.width / 2.0
    half_h = bbox.height / 2.0
    x0 = max(0, int(np.floor(bbox.center_x - half_w)))
    x1 = min(width, int(np.ceil(bbox.center_x + half_w)))
    y0 = max(0, int(np.floor(bbox.center_y - half_h)))
    y1 = min(height, int(np.ceil(bbox.center_y + half_h)))
    if x1 <= x0 or y1 <= y0:
        return np.array([], dtype=depth_img.dtype), (x0, y0, x1, y1)
    return depth_img[y0:y1, x0:x1], (x0, y0, x1, y1)


def _compute_baseline_depth(roi: np.ndarray, cfg: DepthFusionClassConfig) -> float:
    if roi.size == 0:
        return float("nan")
    valid = np.isfinite(roi)
    if cfg.z_min is not None:
        valid &= roi >= cfg.z_min
    if cfg.z_max is not None:
        valid &= roi <= cfg.z_max
    values = roi[valid]
    if not values.size:
        return float("nan")
    return float(np.median(values))


def _build_region_mask(h: int, w: int, region_type: str, band_ratio: float) -> np.ndarray:
    region = np.zeros((h, w), dtype=bool)
    if h == 0 or w == 0:
        return region
    region_type = (region_type or "full").lower()
    ratio = float(np.clip(band_ratio, 0.01, 1.0))
    if region_type == "full":
        region[:, :] = True
    elif region_type == "center":
        half_h = max(1, int(round(h * ratio / 2.0)))
        half_w = max(1, int(round(w * ratio / 2.0)))
        cy = h // 2
        cx = w // 2
        y0 = max(0, cy - half_h)
        y1 = min(h, cy + half_h)
        x0 = max(0, cx - half_w)
        x1 = min(w, cx + half_w)
        region[y0:y1, x0:x1] = True
    elif region_type == "bottom":
        band = max(1, int(round(h * ratio)))
        region[h - band :, :] = True
    elif region_type == "border":
        band = max(1, int(round(min(h, w) * ratio)))
        region[:band, :] = True
        region[:, :band] = True
        region[:, w - band :] = True
    elif region_type == "vertical":
        band = max(1, int(round(w * ratio)))
        cx = w // 2
        x0 = max(0, cx - band // 2)
        x1 = min(w, cx + (band + 1) // 2)
        region[:, x0:x1] = True
    else:
        region[:, :] = True
    return region


def _apply_gate(values: np.ndarray, cfg: DepthFusionClassConfig) -> Tuple[np.ndarray, Dict[str, Any]]:
    if values.size == 0:
        return np.zeros(0, dtype=bool), {"gate_upper": float("nan")}
    median = float(np.median(values))
    q75, q25 = np.percentile(values, [75, 25])
    iqr = q75 - q25
    upper_iqr = median + (cfg.gate_k_iqr * iqr if iqr > 0 else cfg.gate_k_iqr * 0.0)
    percentile_limit = np.percentile(values, min(100.0, max(0.0, cfg.gate_percentile * 100.0)))
    gate_upper = min(upper_iqr, percentile_limit)
    keep = values <= gate_upper
    return keep, {
        "median": median,
        "iqr": float(iqr),
        "gate_upper": float(gate_upper),
        "percentile_limit": float(percentile_limit),
    }


def _cluster_depths(values: np.ndarray, eps: float, min_samples: int) -> List[np.ndarray]:
    if values.size == 0:
        return []
    order = np.argsort(values)
    sorted_vals = values[order]
    clusters: List[np.ndarray] = []
    start = 0
    eps = max(1e-6, float(eps))
    for idx in range(1, sorted_vals.size):
        if (sorted_vals[idx] - sorted_vals[idx - 1]) > eps:
            if (idx - start) >= min_samples:
                clusters.append(sorted_vals[start:idx])
            start = idx
    if (sorted_vals.size - start) >= min_samples:
        clusters.append(sorted_vals[start:])
    return clusters


def _select_cluster(clusters: List[np.ndarray], prefer_closer: bool) -> np.ndarray:
    if not clusters:
        return np.array([])
    if len(clusters) == 1:
        return clusters[0]
    ranked = sorted(
        clusters,
        key=lambda c: (-c.size, float(np.median(c)) if prefer_closer else 0.0, float(np.mean(c))),
    )
    if prefer_closer:
        # Among largest clusters prefer the closest one (smallest median)
        max_size = ranked[0].size
        tied = [c for c in ranked if c.size == max_size]
        return min(tied, key=lambda c: float(np.median(c)))
    return ranked[0]


def _run_constant_ransac(values: np.ndarray, iters: int, tau: float, min_inliers: int) -> np.ndarray:
    if values.size == 0:
        return np.array([])
    best_inliers: np.ndarray = np.array([])
    tau = max(1e-5, float(tau))
    iters = max(1, int(iters))
    for _ in range(iters):
        sample_idx = np.random.randint(0, values.size)
        candidate = values[sample_idx]
        inliers = values[np.abs(values - candidate) <= tau]
        if inliers.size > best_inliers.size:
            best_inliers = inliers
    if best_inliers.size < min_inliers:
        return np.array([])
    return best_inliers


__all__ = [
    "BoundingBox",
    "configure_depth_fusion",
    "estimate_depth_for_detection",
]
