import torch
from iknet import IKNet
import argparse

MAX_X = 1
MAX_Y = 3

RELU_X = 0
RELU_Y = 1

def mainx(x):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    #x -> angle0
    model = IKNet(MAX_X, RELU_X)
    model.load_state_dict(torch.load("iknet_x.pth"))
    model.to(device)
    model.eval()

    #caculate angles from IKNet
    input_ = torch.FloatTensor(x)
    input_ = input_.to(device)
    print(f"input: {input_}")
    output = model(input_)
    print(f"output: {output}")

def mainy(y):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    #y-> angle1,2,3
    model = IKNet(MAX_Y, RELU_Y)
    model.load_state_dict(torch.load("iknet_y.pth"))
    model.to(device)
    model.eval()

    #caculate angles from IKNet
    input_ = torch.FloatTensor(y)
    input_ = input_.to(device)
    print(f"input: {input_}")
    output = model(input_)
    print(f"output: {output}")

if __name__ == "__main__":
    # run example
    # python iknet_testfn.py --x -0.5 --y -0.5
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    args = parser.parse_args()
    
    print('x -> Anglex')
    mainx([args.x])

    print('\n\ny -> Anglex')
    mainy([args.y])
