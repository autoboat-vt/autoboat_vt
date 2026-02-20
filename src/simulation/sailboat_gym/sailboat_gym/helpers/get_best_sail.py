import os.path as osp
import re
import pickle
import pandas as pd
import numpy as np
from glob import glob
from functools import lru_cache

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


def extract_vmc(df):
    df = df.copy()
    df.index = df.index.get_level_values("theta_wind")
    v_max = df["vmc_max"].groupby("theta_wind").max()
    v_max = np.maximum(0, v_max)
    return v_max


def extract_best_sail(df):
    df = df.copy()
    df = df.reset_index()
    df["vmc_max"] = np.maximum(0, df["vmc_max"])

    # we encourage to take the closest sail to 0
    df["abs_diff"] = np.abs(df["theta_sail"] - 0)
    df = df.sort_values("abs_diff")

    max_index = df.groupby("theta_wind")["vmc_max"].idxmax()
    df_max_vmc = df.loc[max_index]

    theta_wind = df_max_vmc["theta_wind"].values
    best_sail = df_max_vmc["theta_sail"].values

    return theta_wind, best_sail


@lru_cache()
def load_best_sail_dict(env_name, wind_velocity=1):
    assert env_name in list(env_by_name.keys()), f"Env {env_name} not found."
    pathname = osp.join(pkl_dir, f"{env_name}_bounds_v_wind_{wind_velocity}.pkl")
    filepaths = sorted(glob(pathname), key=extract_index, reverse=True)
    assert filepaths, (
        f"Error: Please run `python3 scripts/extract_sim_bounds.py --env-name={env_name} --wind-velocity={wind_velocity}` to extract the velocity bounds first."
    )
    filepath = filepaths[0]
    d = pickle.load(open(filepath, "rb"))
    df = dict_to_df(d)
    theta_wind, best_sail = extract_best_sail(df)
    theta_wind, best_sail = np.deg2rad(theta_wind), np.deg2rad(best_sail)  # noqa
    best_sail_dict = dict(zip(theta_wind, best_sail))
    return best_sail_dict


def get_best_sail(env_name, theta_wind, wind_velocity=1):
    theta_wind = theta_wind % (2 * np.pi)
    best_sail_dict = load_best_sail_dict(env_name, wind_velocity)
    best_sail = np.interp(theta_wind, xp=list(best_sail_dict.keys()), fp=list(best_sail_dict.values()))
    return best_sail
