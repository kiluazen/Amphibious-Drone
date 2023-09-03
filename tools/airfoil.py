import pandas as pd
import numpy as np
from scipy.stats import linregress
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
# Plot the linear regression line
# plt.plot(linear_data['Alpha'], slope * linear_data['Alpha'] + intercept, 'r')

# Print the calculated slope
# # print("Linear Slope (Cl-alpha):", slope)


# # Plot the Cl-alpha data
# plt.plot(np.array(cl_alpha_data['Alpha']), np.array(cl_alpha_data['Cl']))

# # Select the linear range of data points (adjust these indices)
# # Plot the linear regression line
# plt.plot(linear_data['Alpha'], slope * linear_data['Alpha'] + intercept, 'r')

# # Print the calculated slope
# print("Linear Slope (Cl-alpha):", slope)

# # Show the plot
# plt.xlabel('Angle of Attack (Alpha)')
# plt.ylabel('Lift Coefficient (Cl)')
# plt.legend(['Cl-alpha data', 'Linear Regression'])
# plt.title('Cl-alpha Curve with Linear Regression')
# plt.grid(True)
# # plt.show()