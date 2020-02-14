from __future__ import division, print_function

from rl_agents.agents.tree_search.abstract import AbstractTreeSearchAgent
from rl_agents.agents.tree_search.graphics.graphics import TreeGraphics
from rl_agents.agents.tree_search.graphics.robust_epc import RobustEPCGraphics
from rl_agents.agents.tree_search.graphics.robust import DiscreteRobustPlannerGraphics, IntervalRobustPlannerGraphics
from rl_agents.agents.tree_search.robust import DiscreteRobustPlannerAgent, IntervalRobustPlannerAgent
from rl_agents.agents.tree_search.robust_epc import RobustEPCAgent


class AgentGraphics(object):
    """
        Graphical visualization of any Agent implementing AbstractAgent.
    """
    @classmethod
    def display(cls, agent, agent_surface, sim_surface=None):
        """
            Display an agent visualization on a pygame surface.

        :param agent: the agent to be displayed
        :param agent_surface: the pygame surface on which the agent is displayed
        :param sim_surface: the pygame surface on which the environment is displayed
        """

        if isinstance(agent, IntervalRobustPlannerAgent):
            IntervalRobustPlannerGraphics.display(agent, agent_surface, sim_surface)
        elif isinstance(agent, DiscreteRobustPlannerAgent):
            DiscreteRobustPlannerGraphics.display(agent, agent_surface, sim_surface)
        elif isinstance(agent, RobustEPCAgent):
            RobustEPCGraphics.display(agent, agent_surface, sim_surface)
        elif isinstance(agent, AbstractTreeSearchAgent):
            TreeGraphics.display(agent, agent_surface)
