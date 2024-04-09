import torch

def get_zoe_model():
    repo = "isl-org/ZoeDepth"
    # Zoe_N
    model_zoe_nk = torch.hub.load(repo, "ZoeD_NK", pretrained=True)

    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
    zoe = model_zoe_nk.to(DEVICE)
    
    return zoe