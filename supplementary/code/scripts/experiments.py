"""
Usage:
  experiments evaluate <environment> <agent> [options]
  experiments -h | --help

Options:
  -h --help              Show this screen.
  --episodes <count>     Number of episodes [default: 1].
  --no-display           Disable environment, agent, and rewards rendering.
  --name-from-config     Name the output folder from the corresponding config files
  --seed <str>           Seed the environments and agents.
  --verbose              Set log level to debug instead of info.
  --repeat <times>       Repeat several times [default: 1].
"""
import datetime
import os
from pathlib import Path
from docopt import docopt

from rl_agents.agents.common.factory import load_environment, load_agent
from rl_agents.trainer import logger
from rl_agents.trainer.evaluation import Evaluation

BENCHMARK_FILE = 'benchmark_summary'
LOGGING_CONFIG = 'configs/logging.json'
VERBOSE_CONFIG = 'configs/verbose.json'


def main():
    opts = docopt(__doc__)
    for _ in range(int(opts['--repeat'])):
        evaluate(opts['<environment>'], opts['<agent>'], opts)


def evaluate(environment_config, agent_config, options):
    """
        Evaluate an agent interacting with an environment.

    :param environment_config: the path of the environment configuration file
    :param agent_config: the path of the agent configuration file
    :param options: the evaluation options
    """
    logger.configure(LOGGING_CONFIG)
    if options['--verbose']:
        logger.configure(VERBOSE_CONFIG)
    env = load_environment(environment_config)
    agent = load_agent(agent_config, env)
    run_directory = None
    if options['--name-from-config']:
        run_directory = "{}_{}_{}".format(Path(agent_config).with_suffix('').name,
                                  datetime.datetime.now().strftime('%Y%m%d-%H%M%S'),
                                  os.getpid())
    options['--seed'] = int(options['--seed']) if options['--seed'] is not None else None
    evaluation = Evaluation(env,
                            agent,
                            run_directory=run_directory,
                            num_episodes=int(options['--episodes']),
                            sim_seed=options['--seed'],
                            recover=False,
                            display_env=not options['--no-display'],
                            display_agent=not options['--no-display'],
                            display_rewards=not options['--no-display'])
    evaluation.test()
    return os.path.relpath(evaluation.monitor.directory)


if __name__ == "__main__":
    main()
