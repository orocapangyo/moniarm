import torch
from iknet import IKNet
import argparse

MAX_Y = 3

def main(y):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    #y-> angle1,2,3
    model = IKNet(MAX_Y)
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
    # python iknet_testfn.py --y 1.0
    parser = argparse.ArgumentParser()
    parser.add_argument("--y", type=float, default=1.0)
    args = parser.parse_args()
    
    print('y -> Anglex')
    main([args.y])
