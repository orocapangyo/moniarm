import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset

class IKDataset(Dataset):
    def __init__(self, kinematics_pose_csv, in_st, out_st, out_len):
        kinematics_pose = pd.read_csv(kinematics_pose_csv)
        #always input width is 1
        input_ = kinematics_pose.iloc[:, in_st: in_st+1].values
        output = kinematics_pose.iloc[:, out_st:out_st+out_len].values   
        self.input_ = torch.tensor(input_, dtype=torch.float32)
        self.output = torch.tensor(output, dtype=torch.float32)

    def __len__(self):
        return len(self.output)

    def __getitem__(self, index):
        return self.input_[index], self.output[index]

class IKNet(nn.Module):  
    pose = 1     
    min_dim = 10
    max_dim = 500
    min_dropout = 0.1
    max_dropout = 0.5

    def __init__(self, out_ch = 3, relu = 1, trial=None):
        super().__init__()
        self.dof = out_ch
        self.input_dims = [400, 300, 200, 100, 50]
        self.dropout = 0.1
        if trial is not None:
            for i in range(0, 5):
                self.input_dims[i] = trial.suggest_int(
                    f"fc{i+2}_input_dim", self.min_dim, self.max_dim
                )
            self.dropout = trial.suggest_float(
                "dropout", self.min_dropout, self.max_dropout
            )

        print(f"input dimentsions: {self.input_dims}")
        print(f"dropout: {self.dropout}")
        layers = []
        input_dim = self.pose
        for output_dim in self.input_dims:
            layers.append(nn.Linear(input_dim, output_dim))
            if relu == 1:
                layers.append(nn.ReLU())
            layers.append(nn.Dropout(self.dropout))
            input_dim = output_dim
        layers.append(nn.Linear(input_dim, self.dof))
        self.layers = nn.Sequential(*layers)

    def forward(self, x):
        return self.layers(x)
