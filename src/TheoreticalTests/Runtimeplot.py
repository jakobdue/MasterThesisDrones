import re
import matplotlib.pyplot as plt

def extract_runtime_data(text):
    drones = []
    times = []

    pattern = r"Drones:\s*(\d+).*?Time:\s*([\d.]+)\s*s"

    for match in re.finditer(pattern, text):
        drones.append(int(match.group(1)))
        times.append(float(match.group(2)))

    return drones, times

joint_astar_text = """
Drones: 1, Steps: 61, Time: 0.002 s, Average length: 12.17
Drones: 2, Steps: 61, Time: 0.051 s, Average length: 12.17
Drones: 3, Steps: 61, Time: 1.489 s, Average length: 12.51
Drones: 4, Steps: 61, Time: 66.028 s, Average length: 12.22
"""

hierarchical_astar_text = """
Drones: 1, Steps: 1, Time: 0.000 s, Average length: 12.17, Room size: 10
Drones: 2, Steps: 2, Time: 0.001 s, Average length: 12.25, Room size: 10
Drones: 3, Steps: 3, Time: 0.001 s, Average length: 12.90, Room size: 10
Drones: 4, Steps: 4, Time: 0.002 s, Average length: 12.42, Room size: 10
Drones: 5, Steps: 5, Time: 0.003 s, Average length: 13.06, Room size: 10
Drones: 6, Steps: 6, Time: 0.004 s, Average length: 12.96, Room size: 10
Drones: 7, Steps: 7, Time: 0.004 s, Average length: 13.12, Room size: 10
Drones: 8, Steps: 8, Time: 0.005 s, Average length: 13.12, Room size: 10
Drones: 9, Steps: 9, Time: 0.005 s, Average length: 13.36, Room size: 10
Drones: 10, Steps: 10, Time: 0.006 s, Average length: 13.37, Room size: 10
Drones: 11, Steps: 11, Time: 0.007 s, Average length: 13.50, Room size: 10
Drones: 12, Steps: 12, Time: 0.008 s, Average length: 13.48, Room size: 10
Drones: 13, Steps: 13, Time: 0.009 s, Average length: 13.64, Room size: 10
Drones: 14, Steps: 14, Time: 0.010 s, Average length: 13.68, Room size: 10
Drones: 15, Steps: 15, Time: 0.010 s, Average length: 13.78, Room size: 10
Drones: 16, Steps: 16, Time: 0.011 s, Average length: 13.85, Room size: 10
Drones: 17, Steps: 17, Time: 0.012 s, Average length: 13.85, Room size: 10
Drones: 18, Steps: 18, Time: 0.013 s, Average length: 13.90, Room size: 10
Drones: 19, Steps: 19, Time: 0.014 s, Average length: 14.01, Room size: 10
Drones: 20, Steps: 20, Time: 0.015 s, Average length: 13.99, Room size: 10
Drones: 21, Steps: 21, Time: 0.016 s, Average length: 14.00, Room size: 10
Drones: 22, Steps: 22, Time: 0.017 s, Average length: 14.16, Room size: 10
Drones: 23, Steps: 23, Time: 0.018 s, Average length: 14.04, Room size: 10
Drones: 24, Steps: 24, Time: 0.019 s, Average length: 14.27, Room size: 10
Drones: 25, Steps: 25, Time: 0.020 s, Average length: 14.14, Room size: 10
Drones: 26, Steps: 26, Time: 0.020 s, Average length: 14.23, Room size: 10
Drones: 27, Steps: 27, Time: 0.022 s, Average length: 14.14, Room size: 10
Drones: 28, Steps: 28, Time: 0.023 s, Average length: 14.17, Room size: 10
Drones: 29, Steps: 29, Time: 0.025 s, Average length: 14.02, Room size: 10
Drones: 30, Steps: 30, Time: 0.026 s, Average length: 14.13, Room size: 10
Drones: 31, Steps: 31, Time: 0.026 s, Average length: 14.07, Room size: 10
Drones: 32, Steps: 32, Time: 0.029 s, Average length: 13.91, Room size: 10
Drones: 33, Steps: 33, Time: 0.029 s, Average length: 13.95, Room size: 10
Drones: 34, Steps: 34, Time: 0.030 s, Average length: 13.95, Room size: 10
Drones: 35, Steps: 35, Time: 0.031 s, Average length: 14.00, Room size: 10
Drones: 36, Steps: 36, Time: 0.031 s, Average length: 13.96, Room size: 10
Drones: 37, Steps: 37, Time: 0.031 s, Average length: 13.91, Room size: 10
Drones: 38, Steps: 38, Time: 0.032 s, Average length: 14.02, Room size: 10
Drones: 39, Steps: 39, Time: 0.031 s, Average length: 13.85, Room size: 10
Drones: 40, Steps: 40, Time: 0.031 s, Average length: 13.94, Room size: 10
Drones: 41, Steps: 41, Time: 0.031 s, Average length: 13.92, Room size: 10
Drones: 42, Steps: 42, Time: 0.031 s, Average length: 13.90, Room size: 10
Drones: 43, Steps: 43, Time: 0.030 s, Average length: 13.90, Room size: 10
Drones: 44, Steps: 44, Time: 0.030 s, Average length: 14.00, Room size: 10
Drones: 45, Steps: 45, Time: 0.030 s, Average length: 13.88, Room size: 10
Drones: 46, Steps: 46, Time: 0.031 s, Average length: 13.86, Room size: 10
Drones: 47, Steps: 47, Time: 0.030 s, Average length: 13.93, Room size: 10
Drones: 48, Steps: 48, Time: 0.028 s, Average length: 13.89, Room size: 10
Drones: 49, Steps: 49, Time: 0.028 s, Average length: 13.92, Room size: 10
Drones: 50, Steps: 50, Time: 0.029 s, Average length: 13.89, Room size: 10
Drones: 51, Steps: 51, Time: 0.029 s, Average length: 13.86, Room size: 10
Drones: 52, Steps: 52, Time: 0.030 s, Average length: 13.89, Room size: 10
Drones: 53, Steps: 53, Time: 0.032 s, Average length: 13.88, Room size: 10
Drones: 54, Steps: 54, Time: 0.035 s, Average length: 13.91, Room size: 10
Drones: 55, Steps: 55, Time: 0.035 s, Average length: 13.84, Room size: 10
Drones: 56, Steps: 56, Time: 0.045 s, Average length: 13.93, Room size: 10
Drones: 57, Steps: 57, Time: 0.035 s, Average length: 13.84, Room size: 10
Drones: 58, Steps: 58, Time: 0.034 s, Average length: 13.83, Room size: 10
Drones: 59, Steps: 59, Time: 0.036 s, Average length: 13.89, Room size: 10
Drones: 60, Steps: 60, Time: 0.035 s, Average length: 13.81, Room size: 10
Drones: 61, Steps: 61, Time: 0.037 s, Average length: 13.83, Room size: 10
Drones: 62, Steps: 62, Time: 0.036 s, Average length: 13.94, Room size: 10
Drones: 63, Steps: 63, Time: 0.037 s, Average length: 13.90, Room size: 10
Drones: 64, Steps: 64, Time: 0.038 s, Average length: 13.84, Room size: 10
Drones: 65, Steps: 65, Time: 0.041 s, Average length: 13.85, Room size: 10
Drones: 66, Steps: 66, Time: 0.041 s, Average length: 13.91, Room size: 10
Drones: 67, Steps: 67, Time: 0.040 s, Average length: 13.89, Room size: 10
Drones: 68, Steps: 68, Time: 0.040 s, Average length: 13.90, Room size: 10
Drones: 69, Steps: 69, Time: 0.043 s, Average length: 13.85, Room size: 10
Drones: 70, Steps: 70, Time: 0.041 s, Average length: 13.91, Room size: 10
Drones: 71, Steps: 71, Time: 0.052 s, Average length: 13.89, Room size: 10
Drones: 72, Steps: 72, Time: 0.055 s, Average length: 13.94, Room size: 10
Drones: 73, Steps: 73, Time: 0.055 s, Average length: 13.87, Room size: 10
Drones: 74, Steps: 74, Time: 0.046 s, Average length: 13.80, Room size: 10
Drones: 75, Steps: 75, Time: 0.044 s, Average length: 13.78, Room size: 10
Drones: 76, Steps: 76, Time: 0.048 s, Average length: 13.79, Room size: 10
Drones: 77, Steps: 77, Time: 0.048 s, Average length: 13.81, Room size: 10
Drones: 78, Steps: 78, Time: 0.064 s, Average length: 13.88, Room size: 10
Drones: 79, Steps: 79, Time: 0.058 s, Average length: 13.85, Room size: 10
Drones: 80, Steps: 80, Time: 0.058 s, Average length: 13.90, Room size: 10
Drones: 81, Steps: 81, Time: 0.058 s, Average length: 13.79, Room size: 10
Drones: 82, Steps: 82, Time: 0.064 s, Average length: 13.82, Room size: 10
Drones: 83, Steps: 83, Time: 0.064 s, Average length: 13.91, Room size: 10
Drones: 84, Steps: 84, Time: 0.071 s, Average length: 13.98, Room size: 10
Drones: 85, Steps: 85, Time: 0.076 s, Average length: 13.87, Room size: 10
Drones: 86, Steps: 86, Time: 0.075 s, Average length: 13.81, Room size: 10
Drones: 87, Steps: 87, Time: 0.073 s, Average length: 13.88, Room size: 10
Drones: 88, Steps: 88, Time: 0.081 s, Average length: 13.82, Room size: 10
Drones: 89, Steps: 89, Time: 0.080 s, Average length: 13.86, Room size: 10
Drones: 90, Steps: 90, Time: 0.076 s, Average length: 13.85, Room size: 10
Drones: 91, Steps: 91, Time: 0.074 s, Average length: 13.78, Room size: 10
Drones: 92, Steps: 92, Time: 0.087 s, Average length: 13.80, Room size: 10
Drones: 93, Steps: 93, Time: 0.080 s, Average length: 13.82, Room size: 10
Drones: 94, Steps: 94, Time: 0.078 s, Average length: 13.79, Room size: 10
Drones: 95, Steps: 95, Time: 0.099 s, Average length: 13.83, Room size: 10
Drones: 96, Steps: 96, Time: 0.104 s, Average length: 13.86, Room size: 10
Drones: 97, Steps: 97, Time: 0.096 s, Average length: 13.87, Room size: 10
Drones: 98, Steps: 98, Time: 0.098 s, Average length: 13.90, Room size: 10
Drones: 99, Steps: 99, Time: 0.098 s, Average length: 13.85, Room size: 10
Drones: 100, Steps: 100, Time: 0.098 s, Average length: 13.82, Room size: 10
Drones: 101, Steps: 101, Time: 0.133 s, Average length: 27.55, Room size: 20
Drones: 102, Steps: 102, Time: 0.129 s, Average length: 27.46, Room size: 20
Drones: 103, Steps: 103, Time: 0.132 s, Average length: 27.54, Room size: 20
Drones: 104, Steps: 104, Time: 0.137 s, Average length: 27.51, Room size: 20
Drones: 105, Steps: 105, Time: 0.137 s, Average length: 27.48, Room size: 20
Drones: 106, Steps: 106, Time: 0.143 s, Average length: 27.46, Room size: 20
Drones: 107, Steps: 107, Time: 0.138 s, Average length: 27.45, Room size: 20
Drones: 108, Steps: 108, Time: 0.141 s, Average length: 27.45, Room size: 20
Drones: 109, Steps: 109, Time: 0.140 s, Average length: 27.36, Room size: 20
Drones: 110, Steps: 110, Time: 0.145 s, Average length: 27.39, Room size: 20
Drones: 111, Steps: 111, Time: 0.142 s, Average length: 27.44, Room size: 20
Drones: 112, Steps: 112, Time: 0.147 s, Average length: 27.40, Room size: 20
Drones: 113, Steps: 113, Time: 0.150 s, Average length: 27.39, Room size: 20
Drones: 114, Steps: 114, Time: 0.148 s, Average length: 27.43, Room size: 20
Drones: 115, Steps: 115, Time: 0.152 s, Average length: 27.41, Room size: 20
Drones: 116, Steps: 116, Time: 0.150 s, Average length: 27.36, Room size: 20
Drones: 117, Steps: 117, Time: 0.154 s, Average length: 27.45, Room size: 20
Drones: 118, Steps: 118, Time: 0.156 s, Average length: 27.43, Room size: 20
Drones: 119, Steps: 119, Time: 0.157 s, Average length: 27.35, Room size: 20
Drones: 120, Steps: 120, Time: 0.163 s, Average length: 27.44, Room size: 20
Drones: 121, Steps: 121, Time: 0.162 s, Average length: 27.39, Room size: 20
Drones: 122, Steps: 122, Time: 0.159 s, Average length: 27.44, Room size: 20
Drones: 123, Steps: 123, Time: 0.163 s, Average length: 27.33, Room size: 20
Drones: 124, Steps: 124, Time: 0.160 s, Average length: 27.40, Room size: 20
Drones: 125, Steps: 125, Time: 0.164 s, Average length: 27.37, Room size: 20
Drones: 126, Steps: 126, Time: 0.169 s, Average length: 27.37, Room size: 20
Drones: 127, Steps: 127, Time: 0.169 s, Average length: 27.35, Room size: 20
Drones: 128, Steps: 128, Time: 0.169 s, Average length: 27.38, Room size: 20
Drones: 129, Steps: 129, Time: 0.170 s, Average length: 27.29, Room size: 20
Drones: 130, Steps: 130, Time: 0.168 s, Average length: 27.32, Room size: 20
Drones: 131, Steps: 131, Time: 0.191 s, Average length: 27.42, Room size: 20
Drones: 132, Steps: 132, Time: 0.178 s, Average length: 27.38, Room size: 20
Drones: 133, Steps: 133, Time: 0.176 s, Average length: 27.33, Room size: 20
Drones: 134, Steps: 134, Time: 0.170 s, Average length: 27.20, Room size: 20
Drones: 135, Steps: 135, Time: 0.199 s, Average length: 27.28, Room size: 20
Drones: 136, Steps: 136, Time: 0.184 s, Average length: 27.25, Room size: 20
Drones: 137, Steps: 137, Time: 0.179 s, Average length: 27.16, Room size: 20
Drones: 138, Steps: 138, Time: 0.206 s, Average length: 27.37, Room size: 20
Drones: 139, Steps: 139, Time: 0.179 s, Average length: 27.15, Room size: 20
Drones: 140, Steps: 140, Time: 0.185 s, Average length: 27.30, Room size: 20
Drones: 141, Steps: 141, Time: 0.189 s, Average length: 27.22, Room size: 20
Drones: 142, Steps: 142, Time: 0.221 s, Average length: 27.29, Room size: 20
Drones: 143, Steps: 143, Time: 0.215 s, Average length: 27.33, Room size: 20
Drones: 144, Steps: 144, Time: 0.208 s, Average length: 27.22, Room size: 20
Drones: 145, Steps: 145, Time: 0.213 s, Average length: 27.21, Room size: 20
Drones: 146, Steps: 146, Time: 0.211 s, Average length: 27.22, Room size: 20
Drones: 147, Steps: 147, Time: 0.199 s, Average length: 27.15, Room size: 20
Drones: 148, Steps: 148, Time: 0.192 s, Average length: 27.09, Room size: 20
Drones: 149, Steps: 149, Time: 0.199 s, Average length: 27.16, Room size: 20
Drones: 150, Steps: 150, Time: 0.241 s, Average length: 27.16, Room size: 20
Drones: 151, Steps: 151, Time: 0.248 s, Average length: 27.23, Room size: 20
Drones: 152, Steps: 152, Time: 0.238 s, Average length: 27.21, Room size: 20
Drones: 153, Steps: 153, Time: 0.213 s, Average length: 27.07, Room size: 20
Drones: 154, Steps: 154, Time: 0.214 s, Average length: 27.01, Room size: 20
Drones: 155, Steps: 155, Time: 0.222 s, Average length: 27.20, Room size: 20
Drones: 156, Steps: 156, Time: 0.249 s, Average length: 27.03, Room size: 20
Drones: 157, Steps: 157, Time: 0.280 s, Average length: 27.19, Room size: 20
Drones: 158, Steps: 158, Time: 0.269 s, Average length: 27.23, Room size: 20
Drones: 159, Steps: 159, Time: 0.251 s, Average length: 27.15, Room size: 20
Drones: 160, Steps: 160, Time: 0.259 s, Average length: 26.99, Room size: 20
Drones: 161, Steps: 161, Time: 0.238 s, Average length: 27.07, Room size: 20
Drones: 162, Steps: 162, Time: 0.266 s, Average length: 27.23, Room size: 20
Drones: 163, Steps: 163, Time: 0.275 s, Average length: 27.04, Room size: 20
Drones: 164, Steps: 164, Time: 0.299 s, Average length: 27.09, Room size: 20
Drones: 165, Steps: 165, Time: 0.295 s, Average length: 27.16, Room size: 20
Drones: 166, Steps: 166, Time: 0.281 s, Average length: 27.10, Room size: 20
Drones: 167, Steps: 167, Time: 0.277 s, Average length: 27.03, Room size: 20
Drones: 168, Steps: 168, Time: 0.294 s, Average length: 26.97, Room size: 20
Drones: 169, Steps: 169, Time: 0.299 s, Average length: 27.08, Room size: 20
Drones: 170, Steps: 170, Time: 0.302 s, Average length: 27.02, Room size: 20
Drones: 171, Steps: 171, Time: 0.305 s, Average length: 27.00, Room size: 20
Drones: 172, Steps: 172, Time: 0.305 s, Average length: 27.08, Room size: 20
Drones: 173, Steps: 173, Time: 0.355 s, Average length: 26.93, Room size: 20
Drones: 174, Steps: 174, Time: 0.318 s, Average length: 26.97, Room size: 20
Drones: 175, Steps: 175, Time: 0.310 s, Average length: 27.11, Room size: 20
Drones: 176, Steps: 176, Time: 0.311 s, Average length: 26.92, Room size: 20
Drones: 177, Steps: 177, Time: 0.347 s, Average length: 27.08, Room size: 20
Drones: 178, Steps: 178, Time: 0.371 s, Average length: 27.08, Room size: 20
Drones: 179, Steps: 179, Time: 0.400 s, Average length: 27.11, Room size: 20
Drones: 180, Steps: 180, Time: 0.403 s, Average length: 27.09, Room size: 20
Drones: 181, Steps: 181, Time: 0.373 s, Average length: 27.16, Room size: 20
Drones: 182, Steps: 182, Time: 0.364 s, Average length: 27.11, Room size: 20
Drones: 183, Steps: 183, Time: 0.361 s, Average length: 27.17, Room size: 20
Drones: 184, Steps: 184, Time: 0.379 s, Average length: 26.95, Room size: 20
Drones: 185, Steps: 185, Time: 0.372 s, Average length: 26.99, Room size: 20
Drones: 186, Steps: 186, Time: 0.470 s, Average length: 27.07, Room size: 20
Drones: 187, Steps: 187, Time: 0.488 s, Average length: 27.21, Room size: 20
Drones: 188, Steps: 188, Time: 0.457 s, Average length: 27.10, Room size: 20
Drones: 189, Steps: 189, Time: 0.478 s, Average length: 27.22, Room size: 20
Drones: 190, Steps: 190, Time: 0.466 s, Average length: 27.16, Room size: 20
Drones: 191, Steps: 191, Time: 0.703 s, Average length: 54.72, Room size: 40
Drones: 192, Steps: 192, Time: 0.669 s, Average length: 54.57, Room size: 40
Drones: 193, Steps: 193, Time: 0.632 s, Average length: 54.68, Room size: 40
Drones: 194, Steps: 194, Time: 0.614 s, Average length: 54.75, Room size: 40
Drones: 195, Steps: 195, Time: 0.620 s, Average length: 54.62, Room size: 40
Drones: 196, Steps: 196, Time: 0.628 s, Average length: 54.57, Room size: 40
Drones: 197, Steps: 197, Time: 0.656 s, Average length: 54.71, Room size: 40
Drones: 198, Steps: 198, Time: 0.644 s, Average length: 54.59, Room size: 40
Drones: 199, Steps: 199, Time: 0.644 s, Average length: 54.59, Room size: 40
Drones: 200, Steps: 200, Time: 0.668 s, Average length: 54.61, Room size: 40
Drones: 201, Steps: 201, Time: 0.677 s, Average length: 54.64, Room size: 40
Drones: 202, Steps: 202, Time: 0.715 s, Average length: 54.65, Room size: 40
Drones: 203, Steps: 203, Time: 0.679 s, Average length: 54.57, Room size: 40
Drones: 204, Steps: 204, Time: 0.685 s, Average length: 54.57, Room size: 40
Drones: 205, Steps: 205, Time: 0.689 s, Average length: 54.60, Room size: 40
Drones: 206, Steps: 206, Time: 0.724 s, Average length: 54.54, Room size: 40
Drones: 207, Steps: 207, Time: 0.680 s, Average length: 54.49, Room size: 40
Drones: 208, Steps: 208, Time: 0.681 s, Average length: 54.47, Room size: 40
Drones: 209, Steps: 209, Time: 0.683 s, Average length: 54.47, Room size: 40
Drones: 210, Steps: 210, Time: 0.701 s, Average length: 54.60, Room size: 40
Drones: 211, Steps: 211, Time: 0.714 s, Average length: 54.50, Room size: 40
Drones: 212, Steps: 212, Time: 0.709 s, Average length: 54.56, Room size: 40
Drones: 213, Steps: 213, Time: 0.713 s, Average length: 54.45, Room size: 40
Drones: 214, Steps: 214, Time: 0.718 s, Average length: 54.42, Room size: 40
Drones: 215, Steps: 215, Time: 0.724 s, Average length: 54.50, Room size: 40
Drones: 216, Steps: 216, Time: 0.744 s, Average length: 54.51, Room size: 40
Drones: 217, Steps: 217, Time: 0.737 s, Average length: 54.42, Room size: 40
Drones: 218, Steps: 218, Time: 0.731 s, Average length: 54.38, Room size: 40
Drones: 219, Steps: 219, Time: 0.752 s, Average length: 54.44, Room size: 40
Drones: 220, Steps: 220, Time: 0.759 s, Average length: 54.37, Room size: 40
Drones: 221, Steps: 221, Time: 0.748 s, Average length: 54.42, Room size: 40
Drones: 222, Steps: 222, Time: 0.743 s, Average length: 54.40, Room size: 40
Drones: 223, Steps: 223, Time: 0.757 s, Average length: 54.40, Room size: 40
Drones: 224, Steps: 224, Time: 0.757 s, Average length: 54.28, Room size: 40
Drones: 225, Steps: 225, Time: 0.759 s, Average length: 54.37, Room size: 40
Drones: 226, Steps: 226, Time: 0.761 s, Average length: 54.30, Room size: 40
Drones: 227, Steps: 227, Time: 0.766 s, Average length: 54.32, Room size: 40
Drones: 228, Steps: 228, Time: 0.769 s, Average length: 54.32, Room size: 40
Drones: 229, Steps: 229, Time: 0.769 s, Average length: 54.26, Room size: 40
Drones: 230, Steps: 230, Time: 0.786 s, Average length: 54.35, Room size: 40
Drones: 231, Steps: 231, Time: 0.792 s, Average length: 54.26, Room size: 40
Drones: 232, Steps: 232, Time: 0.782 s, Average length: 54.34, Room size: 40
Drones: 233, Steps: 233, Time: 0.784 s, Average length: 54.32, Room size: 40
Drones: 234, Steps: 234, Time: 0.793 s, Average length: 54.36, Room size: 40
Drones: 235, Steps: 235, Time: 0.798 s, Average length: 54.22, Room size: 40
Drones: 236, Steps: 236, Time: 0.811 s, Average length: 54.29, Room size: 40
Drones: 237, Steps: 237, Time: 0.815 s, Average length: 54.29, Room size: 40
Drones: 238, Steps: 238, Time: 0.795 s, Average length: 54.26, Room size: 40
Drones: 239, Steps: 239, Time: 0.829 s, Average length: 54.24, Room size: 40
Drones: 240, Steps: 240, Time: 0.816 s, Average length: 54.19, Room size: 40
Drones: 241, Steps: 241, Time: 0.914 s, Average length: 54.28, Room size: 40
Drones: 242, Steps: 242, Time: 0.839 s, Average length: 54.22, Room size: 40
Drones: 243, Steps: 243, Time: 0.819 s, Average length: 54.18, Room size: 40
Drones: 244, Steps: 244, Time: 0.819 s, Average length: 54.23, Room size: 40
Drones: 245, Steps: 245, Time: 0.833 s, Average length: 54.21, Room size: 40
Drones: 246, Steps: 246, Time: 0.829 s, Average length: 54.18, Room size: 40
Drones: 247, Steps: 247, Time: 0.839 s, Average length: 54.22, Room size: 40
Drones: 248, Steps: 248, Time: 0.843 s, Average length: 54.23, Room size: 40
Drones: 249, Steps: 249, Time: 0.833 s, Average length: 54.23, Room size: 40
Drones: 250, Steps: 250, Time: 0.845 s, Average length: 54.18, Room size: 40
Drones: 251, Steps: 251, Time: 0.859 s, Average length: 54.17, Room size: 40
Drones: 252, Steps: 252, Time: 0.853 s, Average length: 54.10, Room size: 40
Drones: 253, Steps: 253, Time: 0.864 s, Average length: 54.21, Room size: 40
Drones: 254, Steps: 254, Time: 0.859 s, Average length: 54.13, Room size: 40
Drones: 255, Steps: 255, Time: 0.853 s, Average length: 54.08, Room size: 40
Drones: 256, Steps: 256, Time: 0.878 s, Average length: 54.09, Room size: 40
Drones: 257, Steps: 257, Time: 0.877 s, Average length: 54.07, Room size: 40
Drones: 258, Steps: 258, Time: 0.883 s, Average length: 54.11, Room size: 40
Drones: 259, Steps: 259, Time: 0.903 s, Average length: 54.22, Room size: 40
Drones: 260, Steps: 260, Time: 0.899 s, Average length: 54.11, Room size: 40
Drones: 261, Steps: 261, Time: 0.909 s, Average length: 54.06, Room size: 40
Drones: 262, Steps: 262, Time: 0.931 s, Average length: 54.17, Room size: 40
Drones: 263, Steps: 263, Time: 0.891 s, Average length: 54.05, Room size: 40
Drones: 264, Steps: 264, Time: 0.888 s, Average length: 54.03, Room size: 40
Drones: 265, Steps: 265, Time: 0.983 s, Average length: 54.11, Room size: 40
Drones: 266, Steps: 266, Time: 0.973 s, Average length: 54.06, Room size: 40
Drones: 267, Steps: 267, Time: 0.974 s, Average length: 54.04, Room size: 40
Drones: 268, Steps: 268, Time: 0.934 s, Average length: 54.00, Room size: 40
Drones: 269, Steps: 269, Time: 1.067 s, Average length: 53.99, Room size: 40
Drones: 270, Steps: 270, Time: 0.930 s, Average length: 53.87, Room size: 40
Drones: 271, Steps: 271, Time: 0.946 s, Average length: 53.94, Room size: 40
Drones: 272, Steps: 272, Time: 0.989 s, Average length: 54.01, Room size: 40
Drones: 273, Steps: 273, Time: 0.931 s, Average length: 53.84, Room size: 40
Drones: 274, Steps: 274, Time: 0.943 s, Average length: 53.87, Room size: 40
Drones: 275, Steps: 275, Time: 1.003 s, Average length: 54.04, Room size: 40
Drones: 276, Steps: 276, Time: 1.210 s, Average length: 53.94, Room size: 40
Drones: 277, Steps: 277, Time: 1.172 s, Average length: 54.08, Room size: 40
Drones: 278, Steps: 278, Time: 1.087 s, Average length: 53.93, Room size: 40
Drones: 279, Steps: 279, Time: 0.993 s, Average length: 53.79, Room size: 40
Drones: 280, Steps: 280, Time: 0.986 s, Average length: 53.83, Room size: 40
Drones: 281, Steps: 281, Time: 0.982 s, Average length: 53.91, Room size: 40
Drones: 282, Steps: 282, Time: 1.026 s, Average length: 53.89, Room size: 40
Drones: 283, Steps: 283, Time: 1.048 s, Average length: 53.83, Room size: 40
Drones: 284, Steps: 284, Time: 1.072 s, Average length: 53.81, Room size: 40
Drones: 285, Steps: 285, Time: 1.315 s, Average length: 53.94, Room size: 40
Drones: 286, Steps: 286, Time: 1.216 s, Average length: 53.78, Room size: 40
Drones: 287, Steps: 287, Time: 1.238 s, Average length: 54.02, Room size: 40
Drones: 288, Steps: 288, Time: 1.079 s, Average length: 53.84, Room size: 40
Drones: 289, Steps: 289, Time: 1.004 s, Average length: 53.71, Room size: 40
Drones: 290, Steps: 290, Time: 1.068 s, Average length: 53.85, Room size: 40
Drones: 291, Steps: 291, Time: 1.129 s, Average length: 53.68, Room size: 40
Drones: 292, Steps: 292, Time: 1.222 s, Average length: 53.75, Room size: 40
Drones: 293, Steps: 293, Time: 1.205 s, Average length: 53.79, Room size: 40
Drones: 294, Steps: 294, Time: 1.285 s, Average length: 54.02, Room size: 40
Drones: 295, Steps: 295, Time: 1.323 s, Average length: 53.83, Room size: 40
Drones: 296, Steps: 296, Time: 1.257 s, Average length: 53.67, Room size: 40
Drones: 297, Steps: 297, Time: 1.333 s, Average length: 53.98, Room size: 40
Drones: 298, Steps: 298, Time: 1.111 s, Average length: 53.58, Room size: 40
Drones: 299, Steps: 299, Time: 1.164 s, Average length: 53.66, Room size: 40
Drones: 300, Steps: 300, Time: 1.433 s, Average length: 53.76, Room size: 40
Drones: 301, Steps: 301, Time: 1.347 s, Average length: 53.54, Room size: 40
Drones: 302, Steps: 302, Time: 1.289 s, Average length: 53.50, Room size: 40
Drones: 303, Steps: 303, Time: 1.339 s, Average length: 53.64, Room size: 40
Drones: 304, Steps: 304, Time: 1.246 s, Average length: 53.51, Room size: 40
Drones: 305, Steps: 305, Time: 1.480 s, Average length: 53.83, Room size: 40
Drones: 306, Steps: 306, Time: 1.490 s, Average length: 53.85, Room size: 40
Drones: 307, Steps: 307, Time: 1.420 s, Average length: 53.72, Room size: 40
Drones: 308, Steps: 308, Time: 1.496 s, Average length: 53.76, Room size: 40
Drones: 309, Steps: 309, Time: 1.473 s, Average length: 53.51, Room size: 40
Drones: 310, Steps: 310, Time: 1.505 s, Average length: 53.86, Room size: 40
Drones: 311, Steps: 311, Time: 1.332 s, Average length: 53.42, Room size: 40
Drones: 312, Steps: 312, Time: 1.214 s, Average length: 53.42, Room size: 40
Drones: 313, Steps: 313, Time: 1.141 s, Average length: 53.53, Room size: 40
Drones: 314, Steps: 314, Time: 1.191 s, Average length: 53.45, Room size: 40
Drones: 315, Steps: 315, Time: 1.549 s, Average length: 53.65, Room size: 40
Drones: 316, Steps: 316, Time: 1.968 s, Average length: 53.72, Room size: 40
Drones: 317, Steps: 317, Time: 1.650 s, Average length: 53.80, Room size: 40
Drones: 318, Steps: 318, Time: 2.432 s, Average length: 53.76, Room size: 40
Drones: 319, Steps: 319, Time: 2.395 s, Average length: 53.47, Room size: 40
Drones: 320, Steps: 320, Time: 1.921 s, Average length: 53.23, Room size: 40
Drones: 321, Steps: 321, Time: 1.304 s, Average length: 53.51, Room size: 40
Drones: 322, Steps: 322, Time: 1.383 s, Average length: 53.55, Room size: 40
Drones: 323, Steps: 323, Time: 1.518 s, Average length: 53.57, Room size: 40
Drones: 324, Steps: 324, Time: 1.559 s, Average length: 53.59, Room size: 40
Drones: 325, Steps: 325, Time: 1.678 s, Average length: 53.59, Room size: 40
Drones: 326, Steps: 326, Time: 1.691 s, Average length: 53.62, Room size: 40
Drones: 327, Steps: 327, Time: 1.600 s, Average length: 53.51, Room size: 40
Drones: 328, Steps: 328, Time: 1.536 s, Average length: 53.60, Room size: 40
Drones: 329, Steps: 329, Time: 1.605 s, Average length: 53.58, Room size: 40
Drones: 330, Steps: 330, Time: 1.382 s, Average length: 53.39, Room size: 40
Drones: 331, Steps: 331, Time: 1.581 s, Average length: 53.51, Room size: 40
Drones: 332, Steps: 332, Time: 1.598 s, Average length: 53.53, Room size: 40
Drones: 333, Steps: 333, Time: 1.486 s, Average length: 53.49, Room size: 40
Drones: 334, Steps: 334, Time: 1.869 s, Average length: 53.58, Room size: 40
Drones: 335, Steps: 335, Time: 1.783 s, Average length: 53.67, Room size: 40
Drones: 336, Steps: 336, Time: 1.779 s, Average length: 53.46, Room size: 40
Drones: 337, Steps: 337, Time: 1.667 s, Average length: 53.66, Room size: 40
Drones: 338, Steps: 338, Time: 1.640 s, Average length: 53.54, Room size: 40
Drones: 339, Steps: 339, Time: 1.809 s, Average length: 53.54, Room size: 40
Drones: 340, Steps: 340, Time: 1.956 s, Average length: 53.62, Room size: 40
Drones: 341, Steps: 341, Time: 1.625 s, Average length: 53.35, Room size: 40
Drones: 342, Steps: 342, Time: 1.732 s, Average length: 53.45, Room size: 40
Drones: 343, Steps: 343, Time: 1.636 s, Average length: 53.34, Room size: 40
Drones: 344, Steps: 344, Time: 1.855 s, Average length: 53.47, Room size: 40
Drones: 345, Steps: 345, Time: 1.961 s, Average length: 53.48, Room size: 40
Drones: 346, Steps: 346, Time: 1.930 s, Average length: 53.64, Room size: 40
Drones: 347, Steps: 347, Time: 1.890 s, Average length: 53.62, Room size: 40
Drones: 348, Steps: 348, Time: 2.086 s, Average length: 53.73, Room size: 40
Drones: 349, Steps: 349, Time: 1.783 s, Average length: 53.41, Room size: 40
Drones: 350, Steps: 350, Time: 1.873 s, Average length: 53.50, Room size: 40
Drones: 351, Steps: 351, Time: 1.701 s, Average length: 53.43, Room size: 40
Drones: 352, Steps: 352, Time: 1.838 s, Average length: 53.37, Room size: 40
Drones: 353, Steps: 353, Time: 1.703 s, Average length: 53.26, Room size: 40
Drones: 354, Steps: 354, Time: 2.109 s, Average length: 53.60, Room size: 40
Drones: 355, Steps: 355, Time: 2.046 s, Average length: 53.52, Room size: 40
Drones: 356, Steps: 356, Time: 2.057 s, Average length: 53.54, Room size: 40
Drones: 357, Steps: 357, Time: 2.027 s, Average length: 53.48, Room size: 40
Drones: 358, Steps: 358, Time: 2.157 s, Average length: 53.49, Room size: 40
Drones: 359, Steps: 359, Time: 1.951 s, Average length: 53.54, Room size: 40
Drones: 360, Steps: 360, Time: 1.872 s, Average length: 53.39, Room size: 40
Drones: 361, Steps: 361, Time: 1.877 s, Average length: 53.52, Room size: 40
Drones: 362, Steps: 362, Time: 2.210 s, Average length: 53.56, Room size: 40
Drones: 363, Steps: 363, Time: 2.311 s, Average length: 53.42, Room size: 40
Drones: 364, Steps: 364, Time: 2.292 s, Average length: 53.63, Room size: 40
Drones: 365, Steps: 365, Time: 2.261 s, Average length: 53.51, Room size: 40
Drones: 366, Steps: 366, Time: 2.260 s, Average length: 53.54, Room size: 40
Drones: 367, Steps: 367, Time: 2.141 s, Average length: 53.58, Room size: 40
Drones: 368, Steps: 368, Time: 2.260 s, Average length: 53.49, Room size: 40
Drones: 369, Steps: 369, Time: 2.388 s, Average length: 53.44, Room size: 40
Drones: 370, Steps: 370, Time: 2.451 s, Average length: 53.32, Room size: 40
Drones: 371, Steps: 371, Time: 2.264 s, Average length: 53.62, Room size: 40
Drones: 372, Steps: 372, Time: 2.219 s, Average length: 53.34, Room size: 40
Drones: 373, Steps: 373, Time: 2.910 s, Average length: 53.59, Room size: 40
Drones: 374, Steps: 374, Time: 2.893 s, Average length: 53.82, Room size: 40
Drones: 375, Steps: 375, Time: 2.954 s, Average length: 53.67, Room size: 40
Drones: 376, Steps: 376, Time: 2.901 s, Average length: 53.62, Room size: 40
Drones: 377, Steps: 377, Time: 2.769 s, Average length: 53.66, Room size: 40
Drones: 378, Steps: 378, Time: 2.685 s, Average length: 53.48, Room size: 40
Drones: 379, Steps: 379, Time: 2.753 s, Average length: 53.57, Room size: 40
Drones: 380, Steps: 380, Time: 2.432 s, Average length: 53.56, Room size: 40
Drones: 381, Steps: 381, Time: 2.500 s, Average length: 53.47, Room size: 40
Drones: 382, Steps: 382, Time: 2.633 s, Average length: 53.45, Room size: 40
Drones: 383, Steps: 383, Time: 4.127 s, Average length: 53.50, Room size: 40
Drones: 384, Steps: 384, Time: 4.455 s, Average length: 53.46, Room size: 40
Drones: 385, Steps: 385, Time: 3.218 s, Average length: 53.56, Room size: 40
Drones: 386, Steps: 386, Time: 3.248 s, Average length: 53.76, Room size: 40
"""

barrier_text = """
Drones: 1, Steps: 35, Time: 0.002 s, Average length: 12.00
Drones: 2, Steps: 35, Time: 0.020 s, Average length: 12.00
Drones: 3, Steps: 35, Time: 0.037 s, Average length: 12.00
Drones: 4, Steps: 35, Time: 0.249 s, Average length: 12.00
Drones: 5, Steps: 35, Time: 0.512 s, Average length: 12.00
Drones: 6, Steps: 35, Time: 0.335 s, Average length: 12.00
Drones: 7, Steps: 35, Time: 1.900 s, Average length: 12.00
Drones: 8, Steps: 35, Time: 1.599 s, Average length: 12.00
Drones: 9, Steps: 35, Time: 0.655 s, Average length: 12.00
Drones: 10, Steps: 35, Time: 2.041 s, Average length: 12.01
Drones: 11, Steps: 35, Time: 4.466 s, Average length: 12.01
Drones: 12, Steps: 35, Time: 4.139 s, Average length: 12.01
Drones: 13, Steps: 35, Time: 2.659 s, Average length: 12.01
Drones: 14, Steps: 35, Time: 6.537 s, Average length: 12.01
Drones: 15, Steps: 35, Time: 5.755 s, Average length: 12.01
Drones: 16, Steps: 35, Time: 7.114 s, Average length: 12.01
Drones: 17, Steps: 35, Time: 6.965 s, Average length: 12.01
Drones: 18, Steps: 35, Time: 11.234 s, Average length: 12.01
Drones: 19, Steps: 35, Time: 14.108 s, Average length: 12.01
Drones: 20, Steps: 35, Time: 7.122 s, Average length: 12.01
Drones: 21, Steps: 35, Time: 15.055 s, Average length: 12.01
Drones: 22, Steps: 35, Time: 22.069 s, Average length: 12.01
Drones: 23, Steps: 35, Time: 8.790 s, Average length: 12.01
Drones: 24, Steps: 35, Time: 23.663 s, Average length: 12.01
Drones: 25, Steps: 35, Time: 17.742 s, Average length: 12.01
Drones: 26, Steps: 35, Time: 15.929 s, Average length: 12.02
Drones: 27, Steps: 35, Time: 17.200 s, Average length: 12.01
Drones: 28, Steps: 35, Time: 30.306 s, Average length: 12.02
Drones: 29, Steps: 35, Time: 25.977 s, Average length: 12.02
Drones: 30, Steps: 35, Time: 43.747 s, Average length: 12.02
"""

joint_rrt_text = """
test: circle Joint, drones: 1, time: 0.10392314600176178
test: circle Joint, drones: 2, time: 0.5016457960009575
test: circle Joint, drones: 3, time: 6.137199249998957
test: circle Joint, drones: 4, time: 11.69494126599966
test: circle Joint, drones: 5, time: 118.10336912700222
"""

hierarchical_rrt_text = """
test: circle hirachical, drones: 1, time: 0.020135369999479735
test: circle hirachical, drones: 2, time: 0.07156172900067759
test: circle hirachical, drones: 3, time: 0.08034566100104712
test: circle hirachical, drones: 4, time: 0.26119408900194685
test: circle hirachical, drones: 5, time: 0.2410213519979152
test: circle hirachical, drones: 6, time: 0.27118478100237553
test: circle hirachical, drones: 7, time: 0.3149040500029514
test: circle hirachical, drones: 8, time: 0.8758298659995489
test: circle hirachical, drones: 9, time: 0.45236617199043394
test: circle hirachical, drones: 10, time: 1.5705779650015756
test: circle hirachical, drones: 11, time: 1.0259911189968989
test: circle hirachical, drones: 12, time: 0.724199315005535
test: circle hirachical, drones: 13, time: 1.6119809999981953
test: circle hirachical, drones: 14, time: 3.2529406910143734
test: circle hirachical, drones: 15, time: 2.8519820109941065
test: circle hirachical, drones: 16, time: 1.4998009099908813
test: circle hirachical, drones: 17, time: 1.9636625789935351
test: circle hirachical, drones: 18, time: 3.16525030000048
test: circle hirachical, drones: 19, time: 2.4908206020081707
test: circle hirachical, drones: 20, time: 6.1172916349969455
test: circle hirachical, drones: 21, time: 3.185178101004567
test: circle hirachical, drones: 22, time: 8.420615951006766
test: circle hirachical, drones: 23, time: 8.39755911899556
test: circle hirachical, drones: 24, time: 13.420791630993335
test: circle hirachical, drones: 25, time: 16.458681888001593
test: circle hirachical, drones: 26, time: 10.239852875001816
test: circle hirachical, drones: 27, time: 38.338320722996286
test: circle hirachical, drones: 28, time: 15.316174361989397
test: circle hirachical, drones: 29, time: 8.769452824999462
test: circle hirachical, drones: 30, time: 74.48450290499022
test: circle hirachical, drones: 31, time: 47.40382294898882
test: circle hirachical, drones: 32, time: 16.73923606300741
test: circle hirachical, drones: 33, time: 174.16079323799931
"""

def extract_rrt_data(text):
    drones = []
    times = []

    pattern = r"drones:\s*(\d+),\s*time:\s*([\d.]+)"

    for match in re.finditer(pattern, text):
        drones.append(int(match.group(1)))
        times.append(float(match.group(2)))

    return drones, times

data = {
    "Joint A*": extract_runtime_data(joint_astar_text),
    "Hierarchical A*": extract_runtime_data(hierarchical_astar_text),
    "Barrier": extract_runtime_data(barrier_text),
    "Joint RRT*": extract_rrt_data(joint_rrt_text),
    "Hierarchical RRT*": extract_rrt_data(hierarchical_rrt_text),
}

plt.figure(figsize=(10, 6))

for label, (drones, times) in data.items():
    if drones and times:
        plt.plot(drones, times, marker="o", markersize=3, linewidth=1.5, label=label)

plt.xlabel("Number of drones")
plt.ylabel("Runtime (s)")
plt.title("Circle Crossing Runtime Comparison")
plt.yscale("log")
plt.grid(True, which="both", linestyle="--", linewidth=0.5)
plt.legend()
plt.tight_layout()
plt.xlim(0, 35)
plt.savefig("circle_runtime_comparison.png", dpi=300)
plt.show()
