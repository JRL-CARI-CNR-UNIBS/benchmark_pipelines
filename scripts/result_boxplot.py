#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import numpy  as np
import math
import matplotlib.pyplot as plt
import seaborn
import pandas as pd
import rospy

def run():
    rospy.init_node('analyze', anonymous=True)

    filename=rospy.get_param('~csv_name')
    figure_folder=rospy.get_param('~figure_folder')
    res=pd.read_csv(filename)
    normalized_res=res.copy()
    boxplot_planning_times=rospy.get_param('~boxplot_planning_times')
    axis_ylim = rospy.get_param('~axis_ylim')


    for q in range(res["query"].min(),res["query"].max()+1):
        min=res[res["query"]==q]["cost"].min()
        normalized_res.loc[normalized_res["query"]==q,"cost"]/=min

    normalized_res["iter"]=1000*(normalized_res["iteration"]*0.001).round()

    tested_planners=np.unique((normalized_res["planner"]).values)

    test_names=np.unique((normalized_res["test_name"]).values)

    if (rospy.has_param('~planner_names_map')):
        planner_names_map=rospy.get_param('~planner_names_map')

        for p in planner_names_map:
            print(p,": ",planner_names_map[p])
            normalized_res = normalized_res.replace(to_replace=p, value=planner_names_map[p])

        planner_names=np.unique((normalized_res["planner"]).values)
    else:
        planner_names=np.unique((normalized_res["planner"]).values)

    a4_dims = (11.7, 8.27)

    test_names=np.unique((normalized_res["test_name"]).values)
    for test_name in test_names:
        db_test=normalized_res[normalized_res["test_name"]==test_name]
        scenarios=np.unique((db_test["scenario"]).values)

        for scenario in scenarios:
            db_scenerario=db_test[db_test["scenario"]==scenario]
            groups=np.unique((db_test["group_name"]).values)

            for group in groups:
                db=db_scenerario[db_scenerario["group_name"]==group]

                name=figure_folder+"/"+test_name+"_"+scenario+"_"+group
                print(name)

                db2=db.loc[db['time'].isin(boxplot_planning_times),:]


                fig, ax = plt.subplots(figsize=a4_dims);
                seaborn.set_context("paper", rc={"font.size":18,"axes.titlesize":18,"axes.labelsize":18,"legend.fontsize":18, "xtick.labelsize": 18, "ytick.labelsize":18})
                g=seaborn.boxplot(hue="planner",y="cost",data=db2,x="time", palette="deep", hue_order=planner_names, showfliers = False)
                #ax.set(ylim=(1, axis_ylim))
                seaborn.set_context("paper", rc={"font.size":18,"axes.titlesize":18,"axes.labelsize":18,"legend.fontsize":18, "xtick.labelsize": 18, "ytick.labelsize":18})
                g.set_xlabel("Planning time [s]", fontsize = 18)
                g.set_ylabel("Normalized length", fontsize = 18)
                g.grid()
                fig.savefig(name+"_box.png",dpi=300, bbox_inches = 'tight')

                for q in range(res["query"].min(),res["query"].max()+1):
                    query_res=db2[db2["query"]==q]

                    fig, ax = plt.subplots(figsize=a4_dims);
                    seaborn.set_context("paper", rc={"font.size":18,"axes.titlesize":18,"axes.labelsize":18,"legend.fontsize":18, "xtick.labelsize": 18, "ytick.labelsize":18})
                    g=seaborn.boxplot(hue="planner",y="cost",data=query_res,x="time", palette="deep", hue_order=planner_names, showfliers = False)
                    #ax.set(ylim=(1, axis_ylim))
                    seaborn.set_context("paper", rc={"font.size":18,"axes.titlesize":18,"axes.labelsize":18,"legend.fontsize":18, "xtick.labelsize": 18, "ytick.labelsize":18})
                    g.set_xlabel("Planning time [s]", fontsize = 18)
                    g.set_ylabel("Normalized length", fontsize = 18)
                    g.grid()
                    fig.savefig(name+"_query_"+str(q)+"_box.png",dpi=300, bbox_inches = 'tight')

if __name__ == '__main__':
    run()
