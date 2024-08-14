import subprocess
import time
import argparse
from typing import List, Tuple, Optional, Dict, Any, Union
import os


def run_commands(
        log_dir: str,
        environment: bool,
        semantic: bool,
        intersection: bool,
        simulation: bool,
        speak: bool
    ):
    """
    run all the commands for generating explanations from the robot's camera images
    Args:
        log_dir: str: the directory where the logs are stored
        environment: bool: whether to run the environment explanation
        semantic: bool: whether to run the explanation for semantic map
        intersection: bool: whether to run the intersection explanation
        simulation: bool: whether the commands are run in simulation mode
        speak: bool: whether to speak the explanations
    """
    commands: List[str] = []
    if environment:
        commands.append(f"python3 test_image.py --log_dir {log_dir} -e {'--sim' if simulation else ''} {'--speak' if speak else ''}")
        print(f"running environment explanation; `{commands[-1]}`")
    if semantic:
        commands.append(f"python3 test_image.py --log_dir {log_dir} -s {'--sim' if simulation else ''}")
        print(f"running semantic explanation; `{commands[-1]}`")
    if intersection:
        commands.append(f"python3 test_image.py --log_dir {log_dir} -i {'--sim' if simulation else ''} {'--speak' if speak else ''}")
        print(f"running intersection explanation; `{commands[-1]}`")

    processes = []
    for command in commands:
        process = subprocess.Popen(command, shell=True)
        processes.append(process)
    
    for process in processes:
        process.wait()  # 全てのプロセスが終了するのを待つ


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log_dir", type=str, required=True)
    parser.add_argument("--environment", "-e", action="store_true")
    parser.add_argument("--semantic", "-s", action="store_true")
    parser.add_argument("--intersection", "-i", action="store_true")
    parser.add_argument("--simulation", "--sim", action="store_true")
    parser.add_argument("--speak", action="store_true")
    args = parser.parse_args()

    # check openai api key
    if os.getenv("OPENAI_API_KEY") is None:
        raise ValueError("OPENAI_API_KEY is not set")

    run_commands(
        log_dir=args.log_dir,
        environment=args.environment,
        semantic=args.semantic,
        intersection=args.intersection,
        simulation=args.simulation,
        speak=args.speak
    )


if __name__ == "__main__":
    main()
