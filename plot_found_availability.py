#!/usr/bin/env python3
"""
Generate plots of FOUND availability across different exclusion buffers.

This script parses uptime_summary.txt files from multiple simulation runs
with different FOUND exclusion buffers and creates plots showing how
availability varies with the exclusion buffer size.
"""

import os
import re
import glob
import numpy as np
import matplotlib.pyplot as plt


def parse_uptime_summary(summary_path):
    """Parse uptime_summary.txt and extract FOUND availability data."""
    data = {
        'overall': None,
        'charging': None,
        'experiment': None,
        'gnss_fix': None,
        'downlink': None,
    }

    if not os.path.exists(summary_path):
        print(f"Warning: {summary_path} not found")
        return data

    with open(summary_path, 'r') as f:
        content = f.read()

    # Extract overall FOUND uptime
    overall_match = re.search(r'FOUND valid uptime \(total\):\s+([\d.]+)%', content)
    if overall_match:
        data['overall'] = float(overall_match.group(1))

    # Extract FOUND uptime by state from STATE-BY-STATE section
    state_patterns = {
        'charging': r'.*?CHARGING:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
        'experiment': r'.*?EXPERIMENT:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
        'gnss_fix': r'.*?GNSS_FIX:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
        'downlink': r'.*?DOWNLINK:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
    }

    for state, pattern in state_patterns.items():
        match = re.search(pattern, content)
        if match:
            data[state] = float(match.group(1))

    return data


def extract_exclusion_buffer_from_path(path):
    """Extract exclusion buffer value from directory path."""
    # Pattern: output_LOST25_FOUND75_FOUND_EXCL30_plots
    match = re.search(r'FOUND_EXCL(\d+)_plots', path)
    if match:
        return int(match.group(1))
    return None


def collect_availability_data(base_dir='simulation'):
    """Collect availability data from all exclusion buffer runs."""
    # Find all directories matching the pattern
    pattern = os.path.join(base_dir, 'output_LOST25_FOUND75_FOUND_EXCL*_plots')
    dirs = sorted(glob.glob(pattern))

    results = []
    for dir_path in dirs:
        excl_buffer = extract_exclusion_buffer_from_path(dir_path)
        if excl_buffer is None:
            continue

        summary_path = os.path.join(dir_path, 'uptime_summary.txt')
        availability = parse_uptime_summary(summary_path)

        results.append({
            'exclusion_buffer': excl_buffer,
            'availability': availability,
            'dir_path': dir_path
        })

    # Sort by exclusion buffer
    results.sort(key=lambda x: x['exclusion_buffer'])
    return results


def plot_found_availability(results, output_dir='simulation'):
    """Generate single plot with all FOUND availability states vs exclusion buffer."""
    if not results:
        print("No data found to plot")
        return

    # Extract data arrays
    excl_buffers = [r['exclusion_buffer'] for r in results]

    # Colors for different states
    colors = {
        'overall': '#1f77b4',      # blue
        'charging': '#2ca02c',     # green
        'experiment': '#ff7f0e',   # orange
        'gnss_fix': '#d62728',     # red
        'downlink': '#9467bd',     # purple
    }

    # Create single large plot with all states
    fig, ax = plt.subplots(figsize=(12, 8))
    for state, color in colors.items():
        avail = [r['availability'][state] for r in results]
        # Filter out None values for plotting
        valid_indices = [i for i, v in enumerate(avail) if v is not None]
        valid_excl = [excl_buffers[i] for i in valid_indices]
        valid_avail = [avail[i] for i in valid_indices]

        if valid_excl:
            marker = 'o' if state == 'overall' else 's'
            ax.plot(valid_excl, valid_avail, marker=marker, color=color,
                   linewidth=2.5, markersize=10, label=state.upper())

    ax.set_xlabel('Exclusion fov [deg]', fontsize=14, )
    ax.set_ylabel('Availability [%]', fontsize=14, )
    ax.set_title('FOUND availability by sat state',
                fontsize=16,)
    ax.grid(True, alpha=0.3)
    ax.set_ylim([0, 100])
    ax.legend(fontsize=12, loc='best')

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'found_availability_combined.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Combined plot saved to: {output_path}")

    plt.show()


def print_summary_table(results):
    """Print a summary table of the data."""
    print("\n" + "="*80)
    print("FOUND AVAILABILITY SUMMARY TABLE")
    print("="*80)
    print(f"{'Excl Buffer':<15} {'Overall':<12} {'CHARGING':<12} {'EXPERIMENT':<12} {'GNSS_FIX':<12} {'DOWNLINK':<12}")
    print("-"*80)

    for r in results:
        excl = r['exclusion_buffer']
        avail = r['availability']
        print(f"{excl:<15} "
              f"{avail['overall'] or 'N/A':<12} "
              f"{avail['charging'] or 'N/A':<12} "
              f"{avail['experiment'] or 'N/A':<12} "
              f"{avail['gnss_fix'] or 'N/A':<12} "
              f"{avail['downlink'] or 'N/A':<12}")

    print("="*80)


def main():
    """Main function."""
    import sys

    # Allow custom base directory
    base_dir = sys.argv[1] if len(sys.argv) > 1 else 'simulation'

    print(f"Searching for data in: {base_dir}")
    results = collect_availability_data(base_dir)

    if not results:
        print("No data found. Make sure you have directories like:")
        print("  output_LOST25_FOUND75_FOUND_EXCL5_plots")
        print("  output_LOST25_FOUND75_FOUND_EXCL10_plots")
        print("  etc.")
        return

    print(f"Found {len(results)} data points:")
    for r in results:
        print(f"  Exclusion buffer: {r['exclusion_buffer']} deg, "
              f"Path: {r['dir_path']}")

    print_summary_table(results)
    plot_found_availability(results, base_dir)


if __name__ == '__main__':
    main()