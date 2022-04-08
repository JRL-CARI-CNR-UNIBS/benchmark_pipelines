#!/usr/bin/env python3


import numpy  as np
import math
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

res=pd.read_csv('~/.ros/res.csv')
normalized_res=res.copy()

for q in range(res["query"].min(),res["query"].max()+1):
    min=res[res["query"]==q]["cost"].min()
    normalized_res.loc[normalized_res["query"]==q,"cost"]/=min

normalized_res=normalized_res[normalized_res["time"]>2]
a4_dims = (11.7, 8.27)
fig, ax = plt.subplots(figsize=a4_dims);


test_name=res["test_name"][0]
n_boot=20
g=sns.lineplot(x="time", y="cost", hue="planner", data=normalized_res, n_boot=n_boot)
g.grid()
g.set_xlabel("Planning time [s]", fontsize = 18)
g.set_ylabel("Normalized length", fontsize = 18)
#ax.set(xlim=(normalized_res["time"].min()-0.01, normalized_res["time"].max()+0.01))
fig.savefig(test_name+".png",dpi=300, bbox_inches = 'tight')

fig, ax = plt.subplots(figsize=a4_dims);
g=sns.lineplot(x="time", y="iteration", hue="planner", data=normalized_res, n_boot=n_boot)
g.grid()
fig.savefig(test_name+"_iter.png",dpi=300, bbox_inches = 'tight')

for q in range(res["query"].min(),res["query"].max()+1):
    fig, ax = plt.subplots(figsize=a4_dims);
    query_res=normalized_res[normalized_res["query"]==q]
    g=sns.lineplot(x="time", y="cost", hue="planner", data=query_res, n_boot=n_boot)
    g.set_xlabel("Planning time [s]", fontsize = 18)
    g.set_ylabel("Normalized length", fontsize = 18)
    g.grid()
    fig.savefig(test_name+"_query_"+str(q)+".png",dpi=300, bbox_inches = 'tight')
