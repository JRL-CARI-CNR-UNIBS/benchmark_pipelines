#!/usr/bin/env python3
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
import seaborn as sns
import pandas as pd
import rospy

def run():

    rospy.init_node('analyze', anonymous=True)

    filename=rospy.get_param('~csv_name')
    figure_folder=rospy.get_param('~figure_folder')
    res=pd.read_csv(filename)


    normalized_res=res.copy()
    for q in range(res["query"].min(),res["query"].max()+1):
        min=res[res["query"]==q]["cost"].min()
        normalized_res.loc[normalized_res["query"]==q,"cost"]/=min

    normalized_res["iter"]=1000*(normalized_res["iteration"]*0.001).round()

    if (rospy.has_param('~planner_names_map')):
        planner_names_map=rospy.get_param('~planner_names_map')

        for p in planner_names_map:
            print(p,": ",planner_names_map[p])
            normalized_res = normalized_res.replace(to_replace=p, value=planner_names_map[p])

        planner_names=np.unique((normalized_res["planner"]).values)
    else:
        planner_names=np.unique((normalized_res["planner"]).values)

    a4_dims = (11.7, 8.27)
    n_boot=20


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

                fig, ax = plt.subplots(figsize=a4_dims);
                g=sns.lineplot(x="time", y="cost", hue="planner", data=db, n_boot=n_boot, hue_order=planner_names)
                g.grid()
                g.set_xlabel("Planning time [s]", fontsize = 18)
                g.set_ylabel("Normalized length", fontsize = 18)
                fig.savefig(name+".png",dpi=300, bbox_inches = 'tight')

                fig, ax = plt.subplots(figsize=a4_dims);
                g=sns.lineplot(x="time", y="iter", hue="planner", data=db, n_boot=n_boot, hue_order=planner_names)
                g.grid()
                g.set_xlabel("Planning time [s]", fontsize = 18)
                g.set_ylabel("Iterations", fontsize = 18)
                fig.savefig(name+"_iter.png",dpi=300, bbox_inches = 'tight')


                fig, ax = plt.subplots(figsize=a4_dims);
                g=sns.lineplot(x="iter", y="cost", hue="planner", data=db, n_boot=n_boot, hue_order=planner_names)
                g.grid()
                g.set_xlabel("Iterations", fontsize = 18)
                g.set_ylabel("Normalized length", fontsize = 18)
                fig.savefig(name+"_cost_iter.png",dpi=300, bbox_inches = 'tight')

                for q in range(res["query"].min(),res["query"].max()+1):
                    query_res=db[db["query"]==q]

                    fig, ax = plt.subplots(figsize=a4_dims);
                    g=sns.lineplot(x="time", y="cost", hue="planner", data=query_res, n_boot=n_boot, hue_order=planner_names)
                    g.set_xlabel("Planning time [s]", fontsize = 18)
                    g.set_ylabel("Normalized length", fontsize = 18)
                    g.grid()
                    fig.savefig(name+"_query_"+str(q)+".png",dpi=300, bbox_inches = 'tight')

                    fig, ax = plt.subplots(figsize=a4_dims);
                    g=sns.lineplot(x="iter", y="cost", hue="planner", data=query_res, n_boot=n_boot, hue_order=planner_names)
                    g.grid()
                    g.set_xlabel("Iterations", fontsize = 18)
                    g.set_ylabel("Normalized length", fontsize = 18)
                    fig.savefig(name+"_query_"+str(q)+"_iter.png",dpi=300, bbox_inches = 'tight')


if __name__ == '__main__':
    run()
