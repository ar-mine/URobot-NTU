import torch
import clip
from PIL import Image

device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

image = torch.cat([preprocess(Image.open("black_pen.jpg")).unsqueeze(0),
                   preprocess(Image.open("black_box.jpg")).unsqueeze(0),
                   preprocess(Image.open("red_paper.jpg")).unsqueeze(0),
                   preprocess(Image.open("silver_pen.jpg")).unsqueeze(0),
                   preprocess(Image.open("white_pen.jpg")).unsqueeze(0),
                   preprocess(Image.open("white_box.jpg")).unsqueeze(0)]).to(device)

text = clip.tokenize(["black box with realsense"]).to(device)

with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)

    logits_per_image, logits_per_text = model(image, text)
    probs = logits_per_text.softmax(dim=-1).cpu().numpy()
    print(probs)
