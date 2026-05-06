from rfdetr import RFDETRSmall

model = RFDETRSmall(pretrain_weights="robosub2026-synthetic-rfdetr_s.pth")
model.export()
