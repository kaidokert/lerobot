#!/usr/bin/env python3
"""
Simple test script to verify Rerun connection from WSL to Windows.
"""

import argparse
import time
import math

try:
    import rerun as rr
    RERUN_AVAILABLE = True
except ImportError:
    print("ERROR: rerun-sdk not installed. Run: pip install rerun-sdk")
    RERUN_AVAILABLE = False
    exit(1)


def test_rerun_connection(host, port=9876):
    """Test connection to Rerun server."""

    rerun_url = f"rerun+http://{host}:{port}/proxy"

    print(f"Testing connection to Rerun at: {rerun_url}")
    print(f"Make sure Rerun is running on Windows with: rerun --bind 0.0.0.0")
    print()

    try:
        # Initialize and connect
        print("1. Initializing Rerun SDK...")
        rr.init("RerunConnectionTest", spawn=False)

        print("2. Connecting to server...")
        rr.connect_grpc(rerun_url)
        print("✓ Connection successful!")
        print()

        # Send test data
        print("3. Sending test data (watch the Rerun viewer)...")
        for i in range(50):
            # Send a sine wave
            value = math.sin(i * 0.1) * 100
            rr.log("test/sine_wave", rr.Scalars(value))

            # Send a counter
            rr.log("test/counter", rr.Scalars(i))

            # Send text
            if i % 10 == 0:
                rr.log("test/messages", rr.TextLog(f"Test message {i}"))

            print(f"   Sent frame {i+1}/50 (sine={value:.2f}, counter={i})", end='\r')
            time.sleep(0.05)

        print()
        print()
        print("✓ Test completed successfully!")
        print()
        print("You should see in Rerun viewer:")
        print("  - test/sine_wave: A sine wave plot")
        print("  - test/counter: A linear increasing plot")
        print("  - test/messages: Text log messages")
        print()

    except ConnectionError as e:
        print(f"✗ Connection failed: {e}")
        print()
        print("Troubleshooting:")
        print("  1. Is Rerun running on Windows? Run: rerun --bind 0.0.0.0")
        print(f"  2. Can you ping the Windows host? Run: ping {host}")
        print(f"  3. Is port {port} open? Check Windows firewall")
        print(f"  4. Try from Windows PowerShell: Test-NetConnection -ComputerName {host} -Port {port}")
        return False

    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test Rerun connection from WSL to Windows")
    parser.add_argument('host', nargs='?', default=None,
                        help='Rerun server host IP (e.g., 192.168.5.222)')
    parser.add_argument('--port', type=int, default=9876,
                        help='Rerun server port (default: 9876)')
    args = parser.parse_args()

    # Auto-detect Windows host if not provided
    if args.host is None:
        import subprocess
        try:
            result = subprocess.run(
                ["ip", "route", "show"],
                capture_output=True,
                text=True,
                check=True
            )
            for line in result.stdout.split('\n'):
                if 'default' in line:
                    host = line.split()[2]
                    print(f"Auto-detected Windows host IP: {host}")
                    args.host = host
                    break
        except Exception as e:
            print(f"Could not auto-detect Windows IP: {e}")
            print("Please provide host IP manually")
            exit(1)

    if args.host is None:
        print("ERROR: Could not determine Windows host IP")
        print("Usage: python test_rerun_connection.py <HOST_IP>")
        exit(1)

    success = test_rerun_connection(args.host, args.port)
    exit(0 if success else 1)
