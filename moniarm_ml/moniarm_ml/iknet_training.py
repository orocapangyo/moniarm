import argparse
import pytorch_pfn_extras as ppe
import pytorch_pfn_extras.training.extensions as extensions
import torch
import torch.optim as optim
from torch.utils.data import DataLoader
from torch.utils.data.dataset import Subset
from iknet import IKDataset, IKNet

LOC_IN_X = 0
LOC_OUT_X = 2
MAX_X = 1
LOC_IN_Y = 1
LOC_OUT_Y = 3
MAX_Y = 3


def get_data_loaders(args, in_st, out_st, out_len):
    dataset = IKDataset(args.kinematics_pose_csv, in_st, out_st, out_len)
    train_size = int(len(dataset) * args.train_val_ratio)
    train_dataset = Subset(dataset, list(range(0, train_size)))
    val_dataset = Subset(dataset, list(range(train_size, len(dataset))))
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, shuffle=True)
    return train_loader, val_loader

def train(manager, args, model, device, train_loader):
    while not manager.stop_trigger:
        model.train()
        for data, target in train_loader:
            with manager.run_iteration(step_optimizers=["main"]):
                data, target = data.to(device), target.to(device)
                output = model(data)
                loss = (output - target).norm()
                ppe.reporting.report({"train/loss": loss.item() / args.batch_size})
                loss.backward()

def validate(args, model, device, data, target):
    model.eval()
    data, target = data.to(device), target.to(device)
    output = model(data)
    loss = (output - target).norm()
    ppe.reporting.report({"val/loss": loss.item() / args.batch_size})

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--kinematics-pose-csv",
        type=str,
        default="./dataset/train/kinematics_pose.csv",
    )

    parser.add_argument("--train-val-ratio", type=float, default=0.8)
    parser.add_argument("--batch-size", type=int, default=10000)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--lr", type=float, default=0.01)
    parser.add_argument("--save-model", action="store_true", default=False)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    #x -> angle0
    model = IKNet(MAX_X)
    print(model)
    model.to(device)
    train_loader, val_loader = get_data_loaders(args, LOC_IN_X, LOC_OUT_X, MAX_X)

    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    trigger = ppe.training.triggers.EarlyStoppingTrigger(
        check_trigger=(3, "epoch"), monitor="val/loss"
    )

    my_extensions = [
        extensions.LogReport(),
        extensions.ProgressBar(),
        extensions.observe_lr(optimizer=optimizer),
        extensions.ParameterStatistics(model, prefix="model"),
        extensions.VariableStatisticsPlot(model),
        extensions.Evaluator(
            val_loader,
            model,
            eval_func=lambda data, target: validate(args, model, device, data, target),
            progress_bar=True,
        ),
        extensions.PlotReport(["train/loss", "val/loss"], "epoch", filename="loss.png"),
        extensions.PrintReport(
            [
                "epoch",
                "iteration",
                "train/loss",
                "lr",
                "val/loss",
            ]
        ),
    ]
    manager = ppe.training.ExtensionsManager(
        model,
        optimizer,
        args.epochs,
        extensions=my_extensions,
        iters_per_epoch=len(train_loader),
        stop_trigger=trigger,
    )
    train(manager, args, model, device, train_loader)

    if args.save_model:
        torch.save(model.state_dict(), "iknet_x.pth")

def mainy():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--kinematics-pose-csv",
        type=str,
        default="./dataset/train/kinematics_pose.csv",
    )
    parser.add_argument("--train-val-ratio", type=float, default=0.8)
    parser.add_argument("--batch-size", type=int, default=10000)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--lr", type=float, default=0.01)
    parser.add_argument("--save-model", action="store_true", default=False)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    #y-> angle1,2,3
    model = IKNet(MAX_Y)
    print(model)
    model.to(device)
    train_loader, val_loader = get_data_loaders(args, LOC_IN_Y, LOC_OUT_Y, MAX_Y)

    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    trigger = ppe.training.triggers.EarlyStoppingTrigger(
        check_trigger=(3, "epoch"), monitor="val/loss"
    )

    my_extensions = [
        extensions.LogReport(),
        extensions.ProgressBar(),
        extensions.observe_lr(optimizer=optimizer),
        extensions.ParameterStatistics(model, prefix="model"),
        extensions.VariableStatisticsPlot(model),
        extensions.Evaluator(
            val_loader,
            model,
            eval_func=lambda data, target: validate(args, model, device, data, target),
            progress_bar=True,
        ),
        extensions.PlotReport(["train/loss", "val/loss"], "epoch", filename="loss.png"),
        extensions.PrintReport(
            [
                "epoch",
                "iteration",
                "train/loss",
                "lr",
                "val/loss",
            ]
        ),
    ]
    manager = ppe.training.ExtensionsManager(
        model,
        optimizer,
        args.epochs,
        extensions=my_extensions,
        iters_per_epoch=len(train_loader),
        stop_trigger=trigger,
    )

    train(manager, args, model, device, train_loader)

    if args.save_model:
        torch.save(model.state_dict(), "iknet_y.pth")   

if __name__ == "__main__":
    main()
    mainy()
