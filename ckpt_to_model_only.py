import torch

model = torch.load('model.pth', map_location=torch.device('cpu'))
print(model.keys())

state_dict = model['state_dict']
torch.save(state_dict, 'model_only.pth')