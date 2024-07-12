import os

from bokeh.embed import file_html
from bokeh.layouts import gridplot
from bokeh.models import Range1d
from bokeh.plotting import figure
from bokeh.resources import CDN
from html2image import Html2Image
from pathlib import Path

from compact_routing_structure import StraightBundle
from delaunay_triangulation import DelaunayTriangulation
from obstacle import PointObstacle
from point import Point
from utils import get_circle

# Minimum visible point diameter and edge width
min_point_radius = 0.5
min_edge_width = 1
t = 1


def visualize(instance, folder, filename, thick_edges=True, show_axes=False, show_delaunay=False):
    """
    Visualizes the given instance.

    :param instance: an Instance object
    :param folder: the folder in which the plot should be saved
    :param filename: the filename of the plot
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
        if thick_edges:
            straight_xs, straight_ys = [], []
            straight_outline_xs, straight_outline_ys = [], []

            elbow_xs, elbow_ys = [], []
            inner_radii, outer_radii = [], []
            start_angles, end_angles = [], []

            done = False
            prev_bundle = edge.elbow_bundle_v1
            current_bundle = prev_bundle.right
            while not done:
                if type(current_bundle) == StraightBundle:
                    # Set segment corners of straight
                    segment_corners = current_bundle.get_corners(t)
                    straight_xs.append([[[corner.x for corner in segment_corners]]])
                    straight_ys.append([[[corner.y for corner in segment_corners]]])

                    # Set outlines of straight
                    straight_outline_xs.append([segment_corners[0].x, segment_corners[1].x])
                    straight_outline_xs.append([segment_corners[2].x, segment_corners[3].x])
                    straight_outline_ys.append([segment_corners[0].y, segment_corners[1].y])
                    straight_outline_ys.append([segment_corners[2].y, segment_corners[3].y])

                    # If we arrive at the elbow bundle corresponding to v2, we are done
                    if current_bundle.right.is_terminal:
                        done = True
                else:
                    # Set center of annular wedge
                    elbow_xs.append(float(current_bundle.point.x))
                    elbow_ys.append(float(current_bundle.point.y))

                    # Set radii of annular wedge
                    inner_radii.append(t * current_bundle.layer_thickness)
                    outer_radii.append(t * (current_bundle.layer_thickness + current_bundle.thickness))

                    # Set angles of annular wedge
                    a1, a2 = current_bundle.get_angles(t)
                    start_angles.append(float(a2))
                    end_angles.append(float(a1))

                # Move to the next bundle of the thick edge
                current_b = current_bundle
                current_bundle = current_bundle.next(prev_bundle)
                prev_bundle = current_b

            # Draw straights
            plot.multi_polygons(straight_xs, straight_ys, line_color=edge.color, fill_color=edge.color)

            # Draw elbows
            plot.annular_wedge(elbow_xs, elbow_ys,
                               inner_radius=inner_radii, outer_radius=outer_radii,
                               start_angle=start_angles, end_angle=end_angles,
                               line_color=edge.color, fill_color=edge.color)

            # Draw outlines of straights
            plot.multi_line(straight_outline_xs, straight_outline_ys, line_color='black')

            # Draw outlines of elbows
            plot.arc(elbow_xs, elbow_ys,
                     radius=inner_radii,
                     start_angle=start_angles, end_angle=end_angles,
                     line_color='black')
            plot.arc(elbow_xs, elbow_ys,
                     radius=outer_radii,
                     start_angle=start_angles, end_angle=end_angles,
                     line_color='black')
        else:
            path = edge.path

            xs = [float(p.x) for p in path]
            ys = [float(p.y) for p in path]

            # Draw thin edge
            plot.line(xs, ys, line_width=min_edge_width, color=edge.color)

    # Draw obstacles
    for obstacle in instance.obstacles:
        if type(obstacle) == PointObstacle:
            plot.circle(float(obstacle.x), float(obstacle.y), color=obstacle.fill_color)
        else:
            path = obstacle.path
            xs = [float(p.x) for p in path]
            ys = [float(p.y) for p in path]
            plot.multi_polygons([[[xs]]], [[[ys]]], line_color=obstacle.stroke_color, fill_color=obstacle.fill_color)

    # Draw vertices
    for vertex in instance.graph.vertices:
        radius = t * vertex.radius if thick_edges else min_point_radius
        plot.circle(float(vertex.x), float(vertex.y), radius=radius, color=vertex.color)

    if show_delaunay:
        # Draw Delaunay triangulation
        xs = []
        ys = []
        for triangle in instance.homotopy.dt.triangles:
            xs.append([[[float(he.origin.x) for he in triangle.half_edges]]])
            ys.append([[[float(he.origin.y) for he in triangle.half_edges]]])
        plot.multi_polygons(xs, ys, line_color='black', fill_alpha=0)

    # Save plot as png
    full_screen_plot = gridplot([[plot]], toolbar_location=None, sizing_mode='stretch_both')
    html = file_html(full_screen_plot, CDN)
    hti = Html2Image()
    hti.screenshot(html_str=html, save_as=f'{instance.name}.png')

    Path(folder).mkdir(parents=True, exist_ok=True)
    os.replace(f'{instance.name}.png', f'{folder}/{filename}.png')
