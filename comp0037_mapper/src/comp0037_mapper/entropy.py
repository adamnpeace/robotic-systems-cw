# python 3
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

line1 = "Entropy Dumb"
line2 = "Entropy Width-Based"

a = pd.read_csv("~/dev/entropy dumb.csv", names=[line1])
b = pd.read_csv("~/dev/entropy width.csv", names=[line2])

length = max(a.shape[0], b.shape[0])

c = a.join(b)

# Add a Time Column
d = pd.DataFrame([i*5 for i in range(length)], columns=["Time"])
d = d.join(c).set_index("Time")

d[line1] = d[line1].fillna(d[line1].min())
d[line2] = d[line2].fillna(d[line2].min())

d.plot(figsize=(16, 9), title="Entropy vs Time").get_figure().savefig("entropy_graph.png")