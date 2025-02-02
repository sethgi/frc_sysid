

from SysIdProblem import SysIdProblem
from config import FieldConfig, RobotConfig
from optim.GTSAMOptimizer import GTSAMOptimizer

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("log_file")
    args = parser.parse_args()
    
    field_config = FieldConfig()
    robot_config = RobotConfig()
    
    problem = SysIdProblem(robot_config, field_config)

    print("Reading Log File")
    problem.process_log(args.log_file)
    print("Done reading log. Setting up optimizer.")
    
    optimizer = GTSAMOptimizer(problem)
    optimizer.setup_optimizer()
    optimized_transform = optimizer.optimize()

    print("Optimized Transform:", optimized_transform)
