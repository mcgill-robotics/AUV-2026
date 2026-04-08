from rfdetr import RFDETRSmall

model = RFDETRSmall(pretrain_weights="robosub2025-synthetic-rfdetr_s.pth")
model.export()
