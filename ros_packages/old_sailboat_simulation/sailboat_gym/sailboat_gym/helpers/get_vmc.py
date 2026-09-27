import click
import os.path as osp
import re
import pickle
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from functools import lru_cache
from glob import glob

from ..envs import env_by_name

current_dir = osp.dirname(osp.abspath(__file__))
pkl_dir = osp.join(current_dir, "..", "pkl")


def extract_index(filepath):
    matchs = re.findall(r"(\d+)", filepath)
    return int(matchs[-1])


def dict_to_df(d):
    dd = {(k1, k2): v2 for k1, v1 in d.items() for k2, v2 in v1.items()}
    df = pd.DataFrame.from_dict(dd, orient="index")
    df.index.names = ["theta_wind", "theta_sail"]
    for col in df.columns:
        if col == "index":
            continue
        for i, new_col in enumerate(["min", "max"]):
            df[f"{col}_{new_col}"] = df[col].apply(lambda x: x[i])
        df = df.drop(col, axis=1)
    return df


def extract_vmc_from_df(df):
    df = df.copy()
    df.index = df.index.get_level_values("theta_wind")
    v_max = df["vmc_max"].groupby("theta_wind").max()
    v_max = np.maximum(0, v_max)
    return v_max.index, v_max.values


# @lru_cache()


def load_vmc_dict(env_name, wind_velocity=1):
    assert env_name in list(env_by_name.keys()), f"Env {env_name} not found."
    pathname = osp.join(pkl_dir, f"{env_name}_bounds_v_wind_{wind_velocity}.pkl")
    filepaths = sorted(glob(pathname), key=extract_index, reverse=True)
    assert filepaths, (
        f"Error: Please run `python3 scripts/extract_sim_bounds.py --env-name={env_name} --wind-velocity={wind_velocity}` to extract the velocity bounds first."
    )
    filepath = filepaths[0]
    d = pickle.load(open(filepath, "rb"))
    df = dict_to_df(d)
    thetas, vmc = extract_vmc_from_df(df)
    thetas = np.deg2rad(thetas)
    vmc_dict = dict(zip(thetas, vmc))
    return vmc_dict


def get_vmc(env_name, theta_wind, wind_velocity=1):
    vmc_dict = load_vmc_dict(env_name, wind_velocity)
    vmc = np.interp(theta_wind, xp=list(vmc_dict.keys()), fp=list(vmc_dict.values()))
    return vmc
