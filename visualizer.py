import math
import numpy as np
import os

from bokeh.embed import file_html
from bokeh.layouts import gridplot
from bokeh.plotting import figure
from bokeh.resources import CDN
from html2image import Html2Image
from pathlib import Path
from scipy.spatial import Delaunay

from obstacle import PointObstacle
from point import Point
from utils import angle, distance

# Minimum visible point diameter and edge width
min_point_diameter = 1
min_edge_width = 1


def visualize(instance, folder, thick_edges=True, show_axes=False, show_delaunay=False):
    """
    Visualizes the given instance.

    :param instance: an Instance object
    :param folder: the folder in which the plot should be saved
    :param thick_edges: whether the instance should be drawn with thick edges
    :param show_axes: whether the axes of the plot should be drawn
    :param show_delaunay: whether the Delaunay triangulation on the vertices and obstacles should be drawn
    """
    plot = figure(match_aspect=True)

    if show_axes:
        plot.axis.visible = True
        plot.grid.visible = True
    else:
        plot.axis.visible = False
        plot.grid.visible = False

    # Draw edges
    for edge in instance.graph.edges:
        path = edge.path
        if thick_edges:
            xs = []
            ys = []
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                if i != 0:
                    plot.circle(float(p1.x), float(p1.y), radius=edge.thickness/2, color=edge.color)

                segment_corners = get_thick_segment_corners(p1, p2, edge.thickness)
                xs.append([[[corner.x for corner in segment_corners]]])
                ys.append([[[corner.y for corner in segment_corners]]])
        else:
            xs = [float(p.x) for p in path]
            ys = [float(p.y) for p in path]

        if thick_edges:
            plot.multi_polygons(xs, ys, line_color=edge.color, fill_color=edge.color)
        else:
            plot.line(xs, ys, line_width=min_edge_width, color=edge.color)

    # Draw obstacles
    for obstacle in instance.obstacles:
        if type(obstacle) == PointObstacle:
            plot.circle(float(obstacle.x), float(obstacle.y), radius=min_point_diameter/2, color=obstacle.fill_color)
        else:
            path = obstacle.path
            xs = [float(p.x) for p in path]
            ys = [float(p.y) for p in path]
            plot.multi_polygons([[[xs]]], [[[ys]]], line_color=obstacle.stroke_color, fill_color=obstacle.fill_color)

    # Draw vertices
    for vertex in instance.graph.vertices:
        size = vertex.diameter if thick_edges else min_point_diameter
        plot.circle(float(vertex.x), float(vertex.y), radius=size/2, color=vertex.color)

    if show_delaunay:
        # Compute Delaunay triangulation on vertices and point obstacles
        vertex_points = [[float(vertex.x), float(vertex.y)] for vertex in instance.graph.vertices]
        obstacle_points = [[float(obstacle.x), float(obstacle.y)] for obstacle in instance.obstacles]
        delaunay_points = np.array(vertex_points + obstacle_points)
        dt = Delaunay(delaunay_points)

        # Draw Delaunay triangulation
        xs = []
        ys = []
        for triangle in dt.simplices:
            xs.append([[[delaunay_points[i][0] for i in triangle]]])
            ys.append([[[delaunay_points[i][1] for i in triangle]]])
        plot.multi_polygons(xs, ys, line_color='black', fill_alpha=0)

    # Save plot as png
    full_screen_plot = gridplot([[plot]], toolbar_location=None, sizing_mode='stretch_both')
    html = file_html(full_screen_plot, CDN)
    hti = Html2Image()
    hti.screenshot(html_str=html, save_as=f'{instance.name}.png')

    Path(folder).mkdir(parents=True, exist_ok=True)
    os.replace(f'{instance.name}.png', f'{folder}/{instance.name}.png')


def get_thick_segment_corners(p1, p2, thickness):
    """
    Returns the four corners of the rectangle forming a thick line segment between points p1 and p2.
    Source: https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
    """
    length = distance(p1, p2)
    theta = angle(p1, p2)
    center = Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)

    tr_x = center.x + length / 2 * math.cos(theta) - thickness / 2 * math.sin(theta)
    tr_y = center.y + length / 2 * math.sin(theta) + thickness / 2 * math.cos(theta)

    tl_x = center.x - length / 2 * math.cos(theta) - thickness / 2 * math.sin(theta)
    tl_y = center.y - length / 2 * math.sin(theta) + thickness / 2 * math.cos(theta)

    bl_x = center.x - length / 2 * math.cos(theta) + thickness / 2 * math.sin(theta)
    bl_y = center.y - length / 2 * math.sin(theta) - thickness / 2 * math.cos(theta)

    br_x = center.x + length / 2 * math.cos(theta) + thickness / 2 * math.sin(theta)
    br_y = center.y + length / 2 * math.sin(theta) - thickness / 2 * math.cos(theta)
    
    return [Point(tr_x, tr_y), Point(tl_x, tl_y), Point(bl_x, bl_y), Point(br_x, br_y)]
