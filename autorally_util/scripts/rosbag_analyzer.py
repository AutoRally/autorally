#! /usr/bin/env python
import argparse
import matplotlib
# matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
# import matplotlib._color_data as mcd
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

from matplotlib import rc
# Make the text look like Latex
rc('font', **{'family': 'serif', 'serif': ['Computer Modern'], 'size': 16})
rc('text', usetex=True)

import numpy as np
import rosbag
import rospy

from mppi_msgs.msg import mppiStatistics
from autorally_msgs.msg import pathIntegralStats # Lap Stats
from autorally_msgs.msg import runstop # runstop
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path


def find_soonest_in_map(input_map, t):
  for t_i in sorted(input_map.keys()):
    if t < t_i:
      return t_i
  return max(input_map.keys())

class RosbagAnalyzerCCRF:

    def __init__(self, file_path, visualize=False, fastest_lap_time=25,
                 is_tube=False, output_prefix="./"):
        self.bag = rosbag.Bag(file_path)
        self.visualize = visualize
        self.is_tube = is_tube
        self.output_prefix = output_prefix

        # Timing info
        self.runstop_periods = []
        self.laps = []
        self.free_energy_color_map = {0: "cyan",
                                      25: "blue",
                                      50: "green",
                                      500: "purple",
                                      2000: "orange",
                                      5000: "pink",  # Second to last color gets dropped
                                      1000000: "red"}
        self.velocity_color_map = {0: "blue",
                                  5.5: "green",
                                  7: "yellow", # Second to last color gets dropped
                                  25: "red"}
        if self.is_tube:
          self.state_usage_color_map = {0: (0, 0, 0, 0.1),  # transparent grey for real state
                                        0.5: "yellow",
                                        1: "red"} # use continuous
        else:
          self.state_usage_color_map = {0: "red",
                                        4: "orange",
                                        5: "green",
                                        6: "#13eac9", # aqua
                                        7: "yellow", # Second to last color gets dropped
                                        8: (0, 0, 0, 0.1)}   # transparent grey for real state
        self.curr_start_control_time = -1
        self.t0 = self.bag.get_start_time()
        self.tf = self.bag.get_end_time() - self.t0
        # self.laps.append((0, pathIntegralStats()))
        for topic, msg, t in self.bag.read_messages(topics=["/lap_stats", "/runstop"]):
            t_float = msg.header.stamp.to_sec() - self.t0
            if topic == "/lap_stats":   # lap stats message
                if msg.stats.lap_time > fastest_lap_time:
                    print("Lap msg time: {}, lap number {}".format(t_float, len(self.laps) + 1))
                    self.laps.append((t_float, msg))
            elif topic == "/runstop":  # runstop message
                # Header stamp is not set in sim, need to use time received instead
                t_float = t.to_sec() - self.t0
                if msg.motionEnabled and self.curr_start_control_time == -1:
                    self.curr_start_control_time = t_float
                elif not msg.motionEnabled and self.curr_start_control_time != -1:
                    if t_float - self.curr_start_control_time < 5:
                      self.curr_start_control_time = -1 # reset to see if there is a good region
                      continue
                    self.runstop_periods.append((self.curr_start_control_time, t_float))
                    self.curr_start_control_time = -1

        if self.curr_start_control_time != -1:
            self.runstop_periods.append((self.curr_start_control_time, self.tf))
        # print("Lap times: {}".format(self.laps))
        print("Runstop Enabled times: {}".format(self.runstop_periods))
        crash_cost = self.laps[0][1].params.crash_coeff

        ignore_laps = []
        # ignore_laps = [16] # RMMPI bag - alpha_autorally0_2020-07-23-14-49-29_0.bag
        # ignore_laps = [] #  MPPI bag - alpha_autorally0_2020-07-23-16-27-57_0.bag
        print("Number of laps before: {}".format(len(self.laps)))
        self.laps = [item for i, item in enumerate(self.laps) if i not in ignore_laps]
        print("Number of laps after: {}".format(len(self.laps)))

        lap_times = np.array([lap_msg_i.stats.lap_time for _, lap_msg_i in self.laps])
        slip_angles_laps = np.array([lap_msg_i.stats.max_slip for _, lap_msg_i in self.laps])

        print("Average Lap time: {} +- {} secs, max_slip overall: {} rads".format(
              np.mean(lap_times), np.std(lap_times), np.max(slip_angles_laps)))

        # Get Boundaries for visualization
        inner_boundary_topics = ["/inner_boundary"]
        outer_boundary_topics = ["/outer_boundary", "/outer_bounday"]
        self.get_boundaries(inner_boundary_topics, outer_boundary_topics)

        # Start creating lists of important values
        self.x = []
        self.y= []
        self.fe_real_plot = []
        self.fe_nom_plot = []
        self.all_fe_real = []
        self.all_fe_nom = []
        self.free_energy_time = []
        self.velocity = []
        self.nominal_state_usage = []

        # Sim System analysis
        for i, ts in enumerate(self.runstop_periods):
            s_time, e_time = ts
            local_laps = self.laps_in_runstop_period(s_time, e_time)
            # First lap
            self.lap_analysis(i, 1, s_time, local_laps[0][0])
            # All the other complete laps
            for j in range(len(self.laps) - 1):
                self.lap_analysis(1, j + 2, self.laps[j][0], self.laps[j + 1][0])
        # Last bit before stopping the vehicle
        self.lap_analysis(1, len(self.laps) + 1, self.laps[-1][0], self.tf)
        # TODO switch between real and sim data analysis with a flag
        # Real System analysis
        # for j in range(len(self.laps) - 1):
        #     self.lap_analysis(1, j + 1, self.laps[j][0], self.laps[j + 1][0])


        print("Average Total Speed: {} +- {} m/s, Top Speed {} m/s".format(
              np.mean(self.velocity), np.std(self.velocity), np.max(self.velocity)))
        self.plot_colored_line(self.x, self.y, self.fe_real_plot, self.free_energy_color_map,
                                "All Laps Free Energy")
        self.plot_colored_line(self.x, self.y, self.velocity, self.velocity_color_map,
                                "All Laps Velocity")
        normalized_fe_real = np.array(self.all_fe_real) / crash_cost
        self.plot_line_graph(self.free_energy_time, normalized_fe_real, "All Laps Free Energy",
                             y_title="Normalized Free Energy", y_min=0, y_max=1)



    def get_boundaries(self, inner_boundary_topics, outer_boundary_topics):
        boundary_topics = inner_boundary_topics + outer_boundary_topics
        self.inner_boundary = []
        self.outer_boundary = []
        for topic, msg, t in self.bag.read_messages(topics=boundary_topics):
            if topic in inner_boundary_topics and not self.inner_boundary:
                for pose_i in msg.poses:
                    position_i = np.array([pose_i.pose.position.x,
                                          pose_i.pose.position.y])
                    self.inner_boundary.append(position_i)
            elif topic in outer_boundary_topics and not self.outer_boundary:
                for pose_i in msg.poses:
                    position_i = np.array([pose_i.pose.position.x,
                                          pose_i.pose.position.y])
                    self.outer_boundary.append(position_i)
            # Once both boundaries are filled, we no longer need to be looking at boundaries
            if self.inner_boundary and self.outer_boundary:
              self.inner_boundary = np.array(self.inner_boundary)
              self.outer_boundary = np.array(self.outer_boundary)
              break


    def laps_in_runstop_period(self, start_runstop_time, end_runstop_time):
        laps = []
        for t, msg in self.laps:
            if start_runstop_time <= t and t <= end_runstop_time:
                laps.append((t, msg))
        return laps


    def lap_analysis(self, take_number, lap_number, start_lap_time, end_lap_time):
        pos_map = {}
        vel_map = {}
        free_energy_map = {}

        state_topics = ["/pose_estimate", "/ground_truth/state", "/particle_filter/pose_estimate"]
        free_energy_topics = ["/mppi_controller/mppiStatistics"]
        for topic, msg, t in self.bag.read_messages(start_time=rospy.Time(start_lap_time + self.t0),
                                                    end_time=rospy.Time(end_lap_time + self.t0),
                                                    topics=state_topics + free_energy_topics):
            t_float = msg.header.stamp.to_sec() - self.t0
            if topic in state_topics:
                # print("Got Odometry")
                # pos_t.append(t_float)
                pos_map[t_float] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
                vel_map[t_float] = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
            elif topic in free_energy_topics:
                free_energy_map[t_float] = msg
        # Make a collection of points for each tier
        new_msg = mppiStatistics()
        new_msg.nominal_state_used
        plot_x = []
        plot_y = []
        real_free_energy_plot = []
        nominal_free_energy_plot = []
        candidate_state = []
        plot_vel = []
        for t in sorted(pos_map.keys()):
            free_energy_t = find_soonest_in_map(free_energy_map, t)
            plot_x.append(pos_map[t][0])
            plot_y.append(pos_map[t][1])
            real_free_energy_plot.append(free_energy_map[free_energy_t].realSys.freeEnergyMean)
            candidate_state.append(free_energy_map[free_energy_t].nominal_state_used)
            nominal_free_energy_plot.append(free_energy_map[free_energy_t].nominalSys.freeEnergyMean)
            plot_vel.append(np.linalg.norm(vel_map[t]))

        lap_real_free_energy = [free_energy_map[t].realSys.freeEnergyMean for t in sorted(free_energy_map.keys())]
        lap_nom_free_energy = [free_energy_map[t].nominalSys.freeEnergyMean for t in sorted(free_energy_map.keys())]

        # print("Starting to graph lap {} free energy, velocity, and nominal states".format(lap_number))
        self.plot_colored_line(plot_x, plot_y, real_free_energy_plot,
                              self.free_energy_color_map,
                              "Lap {} Free Energy".format(lap_number))
        self.plot_colored_line(plot_x, plot_y, plot_vel,
                              self.velocity_color_map,
                              "Lap {} Velocities".format(lap_number))
        self.plot_colored_line(plot_x, plot_y, candidate_state,
                              self.state_usage_color_map,
                              "Lap {} Nominal State Selection".format(lap_number),
                              line_width=5)
        # self.plot_scatter(plot_x, plot_y, candidate_state,
        #                   "Lap {} Nominal State Selection".format(lap_number))
        self.plot_line_graph(sorted(free_energy_map.keys()), lap_real_free_energy,
                              "Lap {} Free Energy".format(lap_number))

        print("Lap {:2d}: Start = {:8.2f} s, End = {:8.2f} s, Time = {:5.4f} s, Avg Vel = {:7.4f} m/s, Max Vel {:7.4f} m/s".format(
          lap_number, start_lap_time, end_lap_time, end_lap_time - start_lap_time, np.mean(plot_vel), np.max(plot_vel)))

        self.x.extend(plot_x)
        self.y.extend(plot_y)
        self.fe_real_plot.extend(real_free_energy_plot)
        self.fe_nom_plot.extend(nominal_free_energy_plot)
        self.all_fe_real.extend(lap_real_free_energy)
        self.all_fe_nom.extend(lap_nom_free_energy)
        self.free_energy_time.extend(sorted(free_energy_map.keys()))
        self.velocity.extend(plot_vel)
        self.nominal_state_usage.extend(candidate_state)

    def plot_line_graph(self, t_array, value_array, title="Free Energy over Time",
                        x_title="t (s)", y_title="Free Energy", y_min=None, y_max=None, show_title=None):
        fig, axs = plt.subplots()
        axs.plot(t_array, value_array)
        # axs.set_title(title)
        axs.set_xlabel(x_title)
        axs.set_ylabel(y_title)

        if y_min is not None and y_max is not None:
          axs.set_ylim(y_min, y_max)

        # Remove the top and right edges of the graph
        axs.spines["right"].set_visible(False)
        axs.spines["top"].set_visible(False)
        axs.yaxis.set_ticks_position("left")
        axs.xaxis.set_ticks_position("bottom")

        plt.savefig(self.output_prefix + "/" + title + " line graph.pdf")
        # axs.set_aspect("equal")
        if self.visualize:
            plt.show(block=True)
        plt.close(fig)

    def plot_colored_line(self, x_array, y_array, val_array, color_dict={},
                          title="Lap Free Energy", x_title="Pos (m)", y_title="Pos (m)",
                          line_width=2):
        # Based on https://matplotlib.org/3.1.1/gallery/lines_bars_and_markers/multicolored_line.html
        colors = []
        number_values = []
        for value in sorted(color_dict.keys()):
            colors.append(color_dict[value])
            number_values.append(value)

        points = np.array([x_array, y_array]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        fig, axs = plt.subplots()

        if not color_dict:
          norm = plt.Normalize(min(val_array), max(val_array))
          cmap = "Wistia"
        else:
            cmap = ListedColormap(colors)
            norm = BoundaryNorm(number_values, cmap.N)

        lc = LineCollection(segments, cmap=cmap, norm=norm)
        lc.set_array(np.array(val_array))
        lc.set_linewidth(line_width)
        line = axs.add_collection(lc)
        fig.colorbar(line, ax=axs)
        min_x_lim = min(self.outer_boundary[:, 0])
        max_x_lim = max(self.outer_boundary[:, 0])
        x_diff = max_x_lim - min_x_lim
        min_x_lim = min_x_lim - 0.01 * x_diff
        max_x_lim = max_x_lim + 0.01 * x_diff
        min_y_lim = min(self.outer_boundary[:, 1])
        max_y_lim = max(self.outer_boundary[:, 1])
        y_diff = max_y_lim - min_y_lim
        min_y_lim = min_y_lim - 0.01 * y_diff
        max_y_lim = max_y_lim + 0.01 * y_diff
        axs.set_xlim(min_x_lim, max_x_lim)
        axs.set_ylim(min_y_lim, max_y_lim)
        axs.plot(self.outer_boundary[:, 0], self.outer_boundary[:, 1], 'k')
        axs.plot(self.inner_boundary[:, 0], self.inner_boundary[:, 1], 'k')

        # axs.set_title(title)
        axs.set_xlabel(x_title)
        axs.set_ylabel(y_title)
        axs.set_aspect("equal")

        # Remove the top and right edges of the graph
        axs.spines["right"].set_visible(False)
        axs.spines["top"].set_visible(False)
        axs.yaxis.set_ticks_position("left")
        axs.xaxis.set_ticks_position("bottom")
        plt.savefig(self.output_prefix + "/" + title + " track.pdf")
        if self.visualize:
            plt.show(block=True)
        plt.close(fig)

    def plot_scatter(self, x_array, y_array, val_array,
                     title="Lap Candidate Selection", x_title="Pos (m)", y_title="Pos (m)"):
        fig, axs = plt.subplots()

        min_x_lim = min(self.outer_boundary[:, 0])
        max_x_lim = max(self.outer_boundary[:, 0])
        x_diff = max_x_lim - min_x_lim
        min_x_lim = min_x_lim - 0.01 * x_diff
        max_x_lim = max_x_lim + 0.01 * x_diff
        min_y_lim = min(self.outer_boundary[:, 1])
        max_y_lim = max(self.outer_boundary[:, 1])
        y_diff = max_y_lim - min_y_lim
        min_y_lim = min_y_lim - 0.01 * y_diff
        max_y_lim = max_y_lim + 0.01 * y_diff
        axs.set_xlim(min_x_lim, max_x_lim)
        axs.set_ylim(min_y_lim, max_y_lim)
        axs.plot(self.outer_boundary[:, 0], self.outer_boundary[:, 1], 'k')
        axs.plot(self.inner_boundary[:, 0], self.inner_boundary[:, 1], 'k')
        val_array = np.array([v if v != 8 else 0 for v in val_array])
        axs.scatter(x_array, y_array, s=8*val_array, c=val_array)

        # axs.set_title(title)
        axs.set_xlabel(x_title)
        axs.set_ylabel(y_title)
        axs.set_aspect("equal")

        # Remove the top and right edges of the graph
        axs.spines["right"].set_visible(False)
        axs.spines["top"].set_visible(False)
        axs.yaxis.set_ticks_position("left")
        axs.xaxis.set_ticks_position("bottom")
        plt.savefig(self.output_prefix + "/" + title + " scatter.pdf")
        if self.visualize:
            plt.show(block=True)
        plt.close(fig)


if __name__ == "__main__":
    print("H")
    parser = argparse.ArgumentParser(description="This script will run through a bag file" +
                                     " and create graphs of the free energy, velocities, " +
                                     "and nominal states selected per lap and for the entire" +
                                     " bag. These are always saved to the output directory")
    parser.add_argument("rosbag_path", type=str, help="Path to rosbag")
    parser.add_argument("--visualize", action="store_true",
                        help="Show the plots of free energy, velocity, and states per lap")
    parser.add_argument("--tube", action="store_true",
                        help="Assume rosbag is Tube-MPPI for appropriately coloring state graphs")
    parser.add_argument("-o", "--output", default="./", type=str, help="Folder to save graphs to")
    args = parser.parse_args()
    new_data = RosbagAnalyzerCCRF(args.rosbag_path, visualize=args.visualize,
                                  is_tube=args.tube, output_prefix=args.output)