import csv
import matplotlib.pyplot as plt
import easygui
import pygame
import datetime
from pygame.color import THECOLORS as COLORS
import numpy as np
from datetime import datetime
import math
def get_dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
class Curve:
    def __init__(self, points, name, multi=False):
        self.points = points
        self.drawn_points = []
        self.name = name
        self.order = 2
        self.multi = multi
        self.fit()
    def fit(self):
        if not self.points:return
        if self.multi:
            xs = [p[0] for p in self.points]
            y = [p[1] for p in self.points]
            self.line = multipolyfit(xs, y, self.order, full=False, model_out=False, powers_out=True)
            print(self.line)
            for i in self.line:
                print(i)
            
        else:
            x = [p[0] for p in self.points]
            y = [p[1] for p in self.points]
            self.line = np.poly1d(np.polyfit(x, y, self.order))
    def check_intersect(self, point):
        for p in self.drawn_points:
            if get_dist(point, [p[0],p[1]])<4:
                return p
    def get_expr(self):
        terms = []
        exponent = len(self.line.c)-1
        for ind, term in enumerate(self.line.c):
            terms.append(f"{term:.3f}"+"*x"*exponent)
            exponent-=1
        return " + ".join(terms)
    def set_order(self, new_order):
        self.order = new_order
        self.fit()
                
def get_choice(title, choices):
    print(title)
    for i,c in enumerate(choices):
        print(f'{i}:{c}, ', end='')
    try:
        index = int(input("Choice #: "))
        return choices[index]

    except (ValueError, IndexError):
        return get_choice(title, choices)

def write_java_file(curve_object):
    #print(curves)
    lines = []
    with open(file_path, 'r') as f:
        orig_lines = f.readlines()
        for i,l in enumerate(orig_lines):
            
            l=l.rstrip()

            if "// CURVE" in l:
                indent = len(l)-len(l.lstrip())
                name = l.split("CURVE:")[1]
                name = name.split(",")[0]
                if name == curve_object.name:
                    now = datetime.now().strftime(r'%I:%M,%m/%d')
                    meta = f"// CURVE:{name},{now}"
                    expr = curve_object.get_expr()
                    l = " "*indent + f"double result = {expr}; {meta}"
            if i!= len(orig_lines)-1: l+="\n"
            lines.append(l)
            
    with open(file_path, 'w') as f:
        f.writelines(lines)

def make_map(in_min, in_max, out_min, out_max):
    in_min+=0.01
    def f(x):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return f

def graph_points(curve, surf, color, label_pos, all_ax_width, draw_frame):
    points = curve.points
    curve.drawn_points = []
    if not points:return
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    x_label_pos =  surf.get_height()-15-15

    x_ax = make_map(min(x), max(x), all_ax_width, surf.get_width()-15)
    y_ax = make_map(min(y), max(y), x_label_pos, 5)
    for ind in range(len(points)):
        dx = x_ax(x[ind])
        dy = y_ax(y[ind])
        curve.drawn_points.append([dx,dy,str(round(x[ind],2)),str(round(y[ind], 1))])
        pygame.draw.circle(surf, color, [dx,dy], 4)
        text = font.render(str(round(y[ind], 1)), 1, color)
        surf.blit(text, [label_pos, dy])

        if(draw_frame):
            pygame.draw.line(surf, [0,0,0], [all_ax_width, 5], [all_ax_width, x_label_pos]) # y axis
            pygame.draw.line(surf, [0,0,0], [all_ax_width, x_label_pos], [surf.get_width()-15, x_label_pos]) # x axis

            xtext = font.render(str(round(x[ind],2)), 1, [0,0,0])
            surf.blit(xtext, [dx, x_label_pos])
            
    text = font.render(curve.name, 1, color)
    surf.blit(text, [label_pos, x_label_pos+10])

    draw_points = []
    for x_val in range(int(min(x))*100, int(max(x)+1)*100, 1):
        x_val/=100
        y_val = curve.line(x_val)
        dx = x_ax(x_val)
        dy = y_ax(y_val)
        draw_points.append([dx,dy])
    pygame.draw.lines(surf, color, False, draw_points)
    



def graph_all_points(curves, surf):
    surf.fill([255,255,230])

    label_pos = 0
    label_width = 40
    all_ax_width = len(curves)*label_width
    
    for ind,name in enumerate(curves):
        use_color = point_colors[ind]
        graph_points(curves[name], surf, use_color, label_pos, all_ax_width, ind==1)
        label_pos+=label_width
        #print(ind, name)
    
def load_curves():
    curves = {}
    current_labels = []

    with open(store_path,'r') as f:
        reader = csv.reader(f, delimiter=',')
        for row in reader:
            if len(row)<2 or len(row)>5:continue
            if row[0] == 'x':
                current_labels = list(row)
                curves = {}
                for label in current_labels:
                    curves[label]=[]
            else:
                x=0
                for i,val in enumerate(row):
                    if val == '':continue
                    val = float(val)
                    if i == 0:
                        x=val
                        continue
                    
                    curves[current_labels[i]].append([x, val])
    del curves['x']
    curve_objects = {}
    for name in curves:
        curve_objects[name] = Curve(curves[name], name)
    return curve_objects
pygame.init()
font = pygame.font.SysFont("Arial", 10)
point_colors = [COLORS['red'], COLORS['green'], COLORS['blue'], COLORS['purple'], COLORS['orange'], COLORS['yellow']]
store_path = r".\passingcurve.csv"
file_path =  r".\src\main\java\frc\robot\subsystems\Shooter.java"
curves = load_curves()

screen = pygame.display.set_mode([640,480])
graphSurf = pygame.surface.Surface([600, 480])
running = True
clock = pygame.time.Clock()
graph_all_points(curves, graphSurf)
disp_point = None
while running:
    clock.tick(30)
    
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        if e.type == pygame.KEYDOWN:
            
            if e.key == pygame.K_r:
                curves=load_curves()
                #graph_all_points(curves, graphSurf)
            if e.key == pygame.K_o:
                if len(curves)>1:
                    choice = easygui.choicebox("which to change order of?", choices=curves.keys())
                else:
                    choice = list(curves.keys())[0]
                if choice is None: continue
                curve = curves[choice]
                
                new_order = easygui.integerbox(f"What order for {curve.name}?")
                if new_order is not None:
                    curve.set_order(new_order)
                graph_all_points(curves, graphSurf)

            if e.key == pygame.K_w:
                if len(curves) >= 2:
                    choices = easygui.multchoicebox("which to write?", choices=curves.keys())
                else:
                    choices = [list(curves.keys())[0]]
                if choices:
                    for choice in choices:
                        write_java_file(curves[choice])
                    easygui.msgbox("Wrote %i curves" % len(choices))
        if e.type == pygame.MOUSEMOTION:
            disp_point = None
            for curve in curves.values():
                found = curve.check_intersect(e.pos)
                if found:
                    disp_point = found
                    break
  
        
    screen.fill([255,255,255])
    graph_all_points(curves, graphSurf)
    if disp_point is not None:
            xtext = font.render(disp_point[2], 1, [0,0,0], [255,255,255])
            graphSurf.blit(xtext, [disp_point[0]+15, disp_point[1]+5])
            ytext = font.render(disp_point[3], 1, [0,0,0], [255,255,255])
            graphSurf.blit(ytext, [disp_point[0]+15, disp_point[1]+20])
    screen.blit(graphSurf, [0,0])
    pygame.display.flip()
    

pygame.quit()
