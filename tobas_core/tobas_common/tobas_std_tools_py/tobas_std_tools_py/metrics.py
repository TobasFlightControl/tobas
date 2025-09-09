import numpy as np
from sklearn.metrics import mean_squared_error


def root_mean_squared_error(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    mse = mean_squared_error(y_true, y_pred)
    rmse = np.sqrt(mse)
    return rmse


def cross_entropy_error(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    cer = -np.sum(y_true * np.log(y_pred + 1e-8))
    return cer
