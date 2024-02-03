import math
import os

from bokeh.embed import file_html
from bokeh.layouts import gridplot
from bokeh.plotting import figure
from bokeh.resources import CDN
from html2image import Html2Image
from pathlib import Path

from utils import angle, distance

# Minimum visible point diameter and edge width
min_point_diameter = 1
min_edge_width = 1


def visualize(graph, obstacles, instance_name, thick_edges=True):
    """
    Visualizes the given instance.

    :param graph: Graph object
    :param obstacles: list of Obstacle objects
    :param instance_name: name of the instance
    :param thick_edges: whether the instance should be drawn with thick edges
    """
    plot = figure()
    plot.axis.visible = False
    plot.grid.visible = False

    # Draw obstacles
    for obstacle in obstacles:
        path = obstacle.path
        if len(path) == 1:
            plot.circle(path[0][0], path[0][1], radius=min_point_diameter/2, color=obstacle.fill_color)
        else:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            plot.multi_polygons([[[xs]]], [[[ys]]], line_color=obstacle.stroke_color, fill_color=obstacle.fill_color)

    # Draw edges
    for edge in graph.edges:
        path = edge.path
        if thick_edges:
            xs = []
            ys = []
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                if i != 0:
                    plot.circle(p1[0], p1[1], radius=edge.thickness/2, color=edge.color)

                segment_corners = get_thick_segment_corners(p1, p2, edge.thickness)
                xs.append([[[corner[0] for corner in segment_corners]]])
                ys.append([[[corner[1] for corner in segment_corners]]])
        else:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
        if thick_edges:
            plot.multi_polygons(xs, ys, line_color=edge.color, fill_color=edge.color)
        else:
            plot.line(xs, ys, line_width=min_edge_width, color=edge.color)

    # Draw vertices
    for vertex in graph.vertices:
        size = vertex.diameter if thick_edges else min_point_diameter
        plot.circle(vertex.x, vertex.y, radius=size/2, color=vertex.color)

    # Save plot as png
    hti = Html2Image()
    full_screen_plot = gridplot([[plot]], toolbar_location=None, sizing_mode='stretch_both')
    html = file_html(full_screen_plot, CDN)
    hti.screenshot(html_str=html, save_as=f'{instance_name}.png')

    if thick_edges:
        path = f'plots/output'
    else:
        path = f'plots/input'

    Path(path).mkdir(parents=True, exist_ok=True)
    os.replace(f'{instance_name}.png', f'{path}/{instance_name}.png')


def get_thick_segment_corners(p1, p2, thickness):
    """
    Returns the four corners of the rectangle forming a thick line segment between p1 and p2.
    Source: https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
    """
    length = distance(p1, p2)
    theta = angle(p1, p2)
    segment_center = [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]

    tr_x = segment_center[0] + length / 2 * math.cos(theta) - thickness / 2 * math.sin(theta)
    tr_y = segment_center[1] + length / 2 * math.sin(theta) + thickness / 2 * math.cos(theta)

    tl_x = segment_center[0] - length / 2 * math.cos(theta) - thickness / 2 * math.sin(theta)
    tl_y = segment_center[1] - length / 2 * math.sin(theta) + thickness / 2 * math.cos(theta)

    bl_x = segment_center[0] - length / 2 * math.cos(theta) + thickness / 2 * math.sin(theta)
    bl_y = segment_center[1] - length / 2 * math.sin(theta) - thickness / 2 * math.cos(theta)

    br_x = segment_center[0] + length / 2 * math.cos(theta) + thickness / 2 * math.sin(theta)
    br_y = segment_center[1] + length / 2 * math.sin(theta) - thickness / 2 * math.cos(theta)
    
    return [[tr_x, tr_y], [tl_x, tl_y], [bl_x, bl_y], [br_x, br_y]]
