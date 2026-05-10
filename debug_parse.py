#!/usr/bin/env python3
import re
import os

# Test with actual file
summary_path = 'simulation/output_LOST25_FOUND75_FOUND_EXCL10_plots/uptime_summary.txt'

print(f"Testing file: {summary_path}")
print(f"File exists: {os.path.exists(summary_path)}")

if os.path.exists(summary_path):
    with open(summary_path, 'r') as f:
        content = f.read()

    print(f"File length: {len(content)} characters")

    # Test overall pattern
    overall_pattern = r'FOUND valid uptime \(total\):\s+([\d.]+)%'
    overall_match = re.search(overall_pattern, content)
    print(f"Overall pattern match: {overall_match is not None}")
    if overall_match:
        print(f"  Overall value: {overall_match.group(1)}")

    # Test state patterns
    state_patterns = {
        'charging': r'CHARGING:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
        'experiment': r'EXPERIMENT:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
        'gnss_fix': r'GNSS_FIX:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
        'downlink': r'DOWNLINK:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
    }

    print("\nState pattern matches:")
    for state, pattern in state_patterns.items():
        match = re.search(pattern, content)
        print(f"  {state.upper():<12}: {'✓ FOUND' if match else '✗ NOT FOUND'}")
        if match:
            print(f"    Value: {match.group(1)}%")

    # Print the actual state section for debugging
    print("\nActual STATE-BY-STATE section:")
    state_section_start = content.find('STATE-BY-STATE AVAILABILITY SUMMARY:')
    if state_section_start >= 0:
        state_section = content[state_section_start:state_section_start+300]
        print(state_section)
else:
    print("File not found!")