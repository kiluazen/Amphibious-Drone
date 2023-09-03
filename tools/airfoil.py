import pandas as pd
import numpy as np
from scipy.stats import linregress
# For a Given airfoil csv file (inside airfoil_data folder) this class can give you cd, lift_slope in future Cl/Cd AoA max value etc..
class airfoil:
    def __init__(self, path) -> None:
        self.path = path
    def a(self):
        airfoilData = pd.read_csv(self.path, skiprows=10)
        cl_alpha = airfoilData[['Alpha','Cl']]
        # Select the linear range of data points (adjust these indices)
        linear_range_start = 25
        linear_range_end = 85
        linear_data = cl_alpha.iloc[linear_range_start:linear_range_end]

        # Perform linear regression
        slope, intercept, r_value, p_value, std_err = linregress(linear_data['Alpha'], linear_data['Cl'])

        linear_range_start = 30
        linear_range_end = 80
        linear_data = cl_alpha.iloc[linear_range_start:linear_range_end]

        # # Perform linear regression
        slope, intercept, r_value, p_value, std_err = linregress(linear_data['Alpha'], linear_data['Cl'])
        return slope*180/np.pi
    def cd(self):
        airfoilData = pd.read_csv(self.path, skiprows=10)
        cd = airfoilData[['Cd']]
        data = cd.iloc[50:100]

        return np.mean(np.array(data))
