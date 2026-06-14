from rfdetr import RFDETRSmall

model = RFDETRSmall(pretrain_weights="./robosub2026-downcam-synth-model.pth")
model.export()
