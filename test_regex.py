#!/usr/bin/env python3
import re

# Test the regex patterns with the actual content
test_content = """
STATE-BY-STATE AVAILABILITY SUMMARY:
   CHARGING: time= 61.86% LOST= 95.93% FOUND=100.00%
  EXPERIMENT: time= 28.63% LOST= 99.81% FOUND= 97.49%
   GNSS_FIX: time=  3.24% LOST= 98.32% FOUND= 35.03%
   DOWNLINK: time=  6.27% LOST= 64.20% FOUND= 49.76%
"""

state_patterns = {
    'charging': r'CHARGING:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
    'experiment': r'EXPERIMENT:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
    'gnss_fix': r'GNSS_FIX:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
    'downlink': r'DOWNLINK:\s*time=\s*[\d.]+%\s*LOST=\s*[\d.]+%\s*FOUND=\s*([\d.]+)%',
}

print("Testing regex patterns:")
print("="*60)

for state, pattern in state_patterns.items():
    match = re.search(pattern, test_content)
    if match:
        print(f"{state.upper():<12} ✓ Found: {match.group(1)}%")
    else:
        print(f"{state.upper():<12} ✗ NOT FOUND")

print("="*60)