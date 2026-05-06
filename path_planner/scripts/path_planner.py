import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

from networkx.algorithms.shortest_paths import astar_path
from typing import List, Tuple, Dict, cast
from scipy.spatial import KDTree