
import sys, csv
import pysindy as ps
import numpy as np
import pandas as pd

if (len(sys.argv) != 2):
	print ("Wrong arguments\n Usage: python csv_to_sindy.py <PATH_TO_CSV>.csv")
	sys.exit(1)

filename = str(sys.argv[1])
print("Reading file:", filename)
csvfilename = filename

with open(csvfilename, 'r') as csvfile:
    df = pd.read_csv(csvfile, sep=r' ', header=0, encoding='ascii', engine='python')

    print("Size before filtering:" + str(df.size))
    df = df.iloc[::10, :] # Only get every 10th row
    df.drop_duplicates(inplace=True, subset=["timestamp_ns"])    
    print("Size after filtering:" + str(df.size))
    print(df.head(10))

    t = df["timestamp_ns"].to_numpy() / 10e+9 # Convert ns to seconds
    t = t - t[0] # Let time start at 0s

    x = df["pos_x"].to_numpy()
    y = df["pos_y"].to_numpy()
    z = df["pos_z"].to_numpy()

    roll = df["roll"].to_numpy()
    pitch = df["pitch"].to_numpy()
    yaw = df["yaw"].to_numpy()

    lin_vel_x = df["linear_vel_x"].to_numpy()
    lin_vel_y = df["linear_vel_y"].to_numpy()
    lin_vel_z = df["linear_vel_z"].to_numpy()
    ang_vel_x = df["angular_vel_x"].to_numpy()
    ang_vel_y = df["angular_vel_y"].to_numpy()
    ang_vel_z = df["angular_vel_z"].to_numpy()

    # Train model
    differentiation_method = ps.FiniteDifference(order=2)
    pollib = ps.PolynomialLibrary(degree=1)
    optimizer = ps.STLSQ(threshold=0.05)

    model = ps.SINDy(feature_library=pollib,
                    differentiation_method=differentiation_method,
                    optimizer=optimizer,
                    discrete_time=True,
                    feature_names=[feature for feature in df.columns[1:]])
    X = np.stack((x, y, z, roll, pitch, yaw, lin_vel_x, lin_vel_y, lin_vel_z, ang_vel_x, ang_vel_y, ang_vel_z), axis=-1)
    print(X.shape)
    print(df.iloc[:, 1:].to_numpy().shape)
    model.fit(X, t=t)
    model.print()
    print(model.score(X, t=t))