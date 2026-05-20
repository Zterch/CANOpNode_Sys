#!/usr/bin/env python3
"""
重力卸载系统数据可视化脚本 v2.0
支持包含 MotorTheoryVel(m/s) 列的新格式CSV文件

使用方法:
    python3 plot_data.py <csv文件路径> [输出图片路径]

示例:
    python3 plot_data.py logdata/gravity_data_20260520_085601.csv
    python3 plot_data.py logdata/gravity_data_20260520_085601.csv output.png
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
import numpy as np

def smooth_data(data, window_size=5):
    """使用移动平均平滑数据"""
    if len(data) < window_size:
        return data
    return np.convolve(data, np.ones(window_size)/window_size, mode='same')

def plot_gravity_data(csv_file, output_file=None):
    """
    读取CSV数据文件并生成可视化图表
    
    参数:
        csv_file: CSV数据文件路径
        output_file: 输出图片路径(可选，默认为csv文件名_analysis.png)
    """
    # 读取CSV文件
    df = pd.read_csv(csv_file, skipinitialspace=True)
    
    # 打印列名用于调试
    print("CSV列名:", [c.strip() for c in df.columns])
    
    # 使用第一列作为时间
    time_col = df.columns[0]
    
    # 转换时间戳为秒（相对时间）
    df['Time_sec'] = pd.to_datetime(df[time_col].str.strip(), format='%H:%M:%S.%f')
    df['Time_sec'] = (df['Time_sec'] - df['Time_sec'].iloc[0]).dt.total_seconds()
    
    # 获取其他列名（去除首尾空格）
    cols = {c.strip(): c for c in df.columns}
    
    # 检查是否有理论速度列
    has_theory_vel = 'MotorTheoryVel(m/s)' in cols
    
    pressure_col = cols['Pressure(kg)']
    rope_len_col = cols['RopeLength(m)']
    motor_speed_col = cols['MotorSpeed(rpm)']
    motor_vel_col = cols['MotorLinearVel(m/s)']
    motor_pos_col = cols['MotorPosition(m)']
    rope_raw_col = cols['RopeVelocityRaw(m/s)']
    rope_filt_col = cols['RopeVelocityFiltered(m/s)']
    
    # 平滑处理关键数据
    df['Pressure_smooth'] = smooth_data(df[pressure_col].values)
    df['RopeVel_smooth'] = smooth_data(df[rope_filt_col].values)
    df['MotorVel_smooth'] = smooth_data(df[motor_vel_col].values)
    
    # 创建图表 - 根据是否有理论速度列调整布局
    if has_theory_vel:
        fig, axes = plt.subplots(5, 1, figsize=(16, 20))
        fig.suptitle('Gravity Unload System Data Analysis v2.0', fontsize=18, fontweight='bold')
    else:
        fig, axes = plt.subplots(4, 1, figsize=(16, 16))
        fig.suptitle('Gravity Unload System Data Analysis', fontsize=18, fontweight='bold')
    
    # 1. 压力曲线
    ax = axes[0]
    ax.plot(df['Time_sec'], df[pressure_col], 'b-', linewidth=0.5, alpha=0.3, label='Raw')
    ax.plot(df['Time_sec'], df['Pressure_smooth'], 'b-', linewidth=1.5, label='Smoothed')
    ax.set_ylabel('Pressure (kg)', fontsize=12)
    ax.set_title('Pressure Sensor Data', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 2. 位置数据
    ax = axes[1]
    ax.plot(df['Time_sec'], df[rope_len_col], 'g-', linewidth=1, label='RopeLength(m)')
    ax.plot(df['Time_sec'], df[motor_pos_col], 'r-', linewidth=1, label='MotorPosition(m)')
    ax.set_ylabel('Position (m)', fontsize=12)
    ax.set_title('Position Data', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 3. 速度对比（关键图表）
    ax = axes[2]
    ax.plot(df['Time_sec'], df[rope_filt_col], 'b-', linewidth=0.5, alpha=0.3)
    ax.plot(df['Time_sec'], df['RopeVel_smooth'], 'b-', linewidth=2, label='RopeVelocityFiltered(m/s)')
    ax.plot(df['Time_sec'], df[motor_vel_col], 'r-', linewidth=0.5, alpha=0.3)
    ax.plot(df['Time_sec'], df['MotorVel_smooth'], 'r-', linewidth=2, label='MotorLinearVel(m/s)')
    
    # 如果有理论速度列，添加理论速度曲线
    if has_theory_vel:
        theory_col = cols['MotorTheoryVel(m/s)']
        df['Theory_smooth'] = smooth_data(df[theory_col].values)
        ax.plot(df['Time_sec'], df[theory_col], 'g-', linewidth=0.5, alpha=0.3)
        ax.plot(df['Time_sec'], df['Theory_smooth'], 'g-', linewidth=2, label='MotorTheoryVel(m/s)')
    
    ax.set_ylabel('Velocity (m/s)', fontsize=12)
    ax.set_title('Velocity Comparison (Smoothed)', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 4. 电机转速
    ax = axes[3]
    ax.plot(df['Time_sec'], df[motor_speed_col], 'm-', linewidth=1, label='MotorSpeed(rpm)')
    ax.set_ylabel('Speed (rpm)', fontsize=12)
    ax.set_title('Motor Speed', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 5. 速度比例分析（如果有理论速度）
    if has_theory_vel:
        ax = axes[4]
        theory_col = cols['MotorTheoryVel(m/s)']
        
        # 计算实际速度与理论速度的比例
        mask = (abs(df[theory_col]) > 0.01) & (abs(df[motor_vel_col]) > 0.01)
        if mask.sum() > 0:
            ratio = df.loc[mask, motor_vel_col] / df.loc[mask, theory_col]
            ratio_smooth = smooth_data(ratio.values, window_size=10)
            
            ax.plot(df.loc[mask, 'Time_sec'], ratio, 'b-', linewidth=0.3, alpha=0.3)
            ax.plot(df.loc[mask, 'Time_sec'], ratio_smooth, 'b-', linewidth=2, label='Actual/Theory Ratio')
            ax.axhline(y=1.0, color='r', linestyle='--', linewidth=1, label='Target (1.0)')
            ax.set_ylabel('Ratio', fontsize=12)
            ax.set_xlabel('Time (seconds)', fontsize=12)
            ax.set_title('Motor Velocity Tracking Ratio (Actual/Theory)', fontsize=14)
            ax.grid(True, alpha=0.3)
            ax.legend()
    else:
        axes[3].set_xlabel('Time (seconds)', fontsize=12)
    
    plt.tight_layout()
    
    # 确定输出路径
    if output_file is None:
        base_name = os.path.splitext(os.path.basename(csv_file))[0]
        output_file = f"{base_name}_analysis.png"
    
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Chart saved to: {output_file}")
    
    # 统计分析
    print(f"\nData Collection Statistics:")
    print(f"  Total samples: {len(df)}")
    
    time_diff = df['Time_sec'].diff().dropna()
    print(f"  Average Period: {time_diff.mean()*1000:.2f} ms")
    print(f"  Average Frequency: {1/time_diff.mean():.2f} Hz")
    
    # 速度跟踪分析
    if has_theory_vel:
        theory_col = cols['MotorTheoryVel(m/s)']
        mask = (abs(df[theory_col]) > 0.01) & (abs(df[motor_vel_col]) > 0.01)
        if mask.sum() > 0:
            ratio = df.loc[mask, motor_vel_col] / df.loc[mask, theory_col]
            print(f"\nVelocity Tracking Analysis:")
            print(f"  Mean ratio (Actual/Theory): {ratio.mean():.3f}")
            print(f"  Std ratio: {ratio.std():.3f}")
            print(f"  Min ratio: {ratio.min():.3f}")
            print(f"  Max ratio: {ratio.max():.3f}")
            
            # 计算跟踪误差
            error = abs(df.loc[mask, motor_vel_col] - df.loc[mask, theory_col])
            print(f"\nTracking Error:")
            print(f"  Mean error: {error.mean():.3f} m/s")
            print(f"  Max error: {error.max():.3f} m/s")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_data.py <csv_file> [output_png]")
        print("Example: python3 plot_data.py logdata/gravity_data_20260520_085601.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found!")
        sys.exit(1)
    
    plot_gravity_data(csv_file, output_file)
