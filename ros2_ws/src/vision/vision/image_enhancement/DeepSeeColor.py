# Copyright 2023 Stewart Jamieson, Woods Hole Oceanographic Institution
# This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License, version 3, as published by the Free Software Foundation.
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.
# You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>. 
import torch
import torch.nn as nn

class BackscatterNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.backscatter_conv = nn.Conv2d(1, 3, 1, bias=False)
        self.residual_conv = nn.Conv2d(1, 3, 1, bias=False)
        nn.init.uniform_(self.backscatter_conv.weight, 0, 5)
        nn.init.uniform_(self.residual_conv.weight, 0, 5)
        self.B_inf = nn.Parameter(torch.rand(3, 1, 1))
        self.J_prime = nn.Parameter(torch.rand(3, 1, 1))
        self.sigmoid = nn.Sigmoid()
        self.relu = nn.ReLU()

    def forward(self, image, depth):
        beta_b_conv = self.relu(self.backscatter_conv(depth))
        beta_d_conv = self.relu(self.residual_conv(depth))
        Bc = self.B_inf * (1 - torch.exp(-beta_b_conv)) + self.J_prime * torch.exp(-beta_d_conv)
        backscatter = self.sigmoid(Bc)
        backscatter_masked = backscatter * (depth > 0.).repeat(1, 3, 1, 1)
        direct = image - backscatter_masked
        return direct, backscatter


class DeattenuateNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.attenuation_conv = nn.Conv2d(1, 6, 1, bias=False)
        nn.init.uniform_(self.attenuation_conv.weight, 0, 5)
        self.attenuation_coef = nn.Parameter(torch.rand(6, 1, 1))
        self.relu = nn.ReLU()
        self.wb = nn.Parameter(torch.rand(1, 1, 1))
        nn.init.constant_(self.wb, 1)
        self.output_act = nn.Sigmoid()

    def forward(self, direct, depth):
        attn_conv = torch.exp(-self.relu(self.attenuation_conv(depth)))
        beta_d = torch.stack(tuple(
            torch.sum(attn_conv[:, i:i + 2, :, :] * self.relu(self.attenuation_coef[i:i + 2]), dim=1) for i in
            range(0, 6, 2)), dim=1)
        f = torch.exp(torch.clamp(beta_d * depth, 0, float(torch.log(torch.tensor([3.])))))
        f_masked = f * ((depth == 0.) / f + (depth > 0.))
        J = f_masked * direct * self.wb
        nanmask = torch.isnan(J)
        if torch.any(nanmask):
            print("Warning! NaN values in J")
            J[nanmask] = 0
        return f_masked, J
        