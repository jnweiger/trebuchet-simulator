#!/usr/bin/python
#
# (C) Copyright 2011, jw@suse.de, licensed as below.
#
# Simulation of shooting a trebuchet type catapult.
# I actually own two of these beasts, and wanted to demo the math 
# to my scientific audience. 
#
# My work is derived from the testbed example test_Domino.py, this is their original
# License header, its terms also applies to my code.
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
# Python version Copyright (c) 2008 kne / sirkne at gmail dot com
# 
# Implemented using the pybox2d SWIG interface for Box2D (pybox2d.googlecode.com)
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.
#
# 2011-08-25, jw, V0.2  sling_trace and proj_trace added.
# 2011-08-26, jw, V0.3  instructions added, projectile reporting added.
#                       Zoom center shifted. Some joint friction added.
# 2011-10-17, jw, V0.4  Tracking craters, old_proj_trace, adjustable 
#                       release_angle. movable arm before shooting.

# study file:///usr/share/doc/packages/python-box2d/manual.htm

# From: X11:Sugar devel:languages:python
# Requires: python-box2d python-pygame python-pgu

from pygame_framework import *
import math

VERSION = 0.4

## definitions of the trebuchet:
l1      = 0.4           # 0.4  [m], length of short arm
l2      = 1.6           # 1.6  [m], length of long arm
l3      = 1.5           # 1.5  [m], length of sling
l4      = 0.5           # 0.5  [m], distance box axis to center of counterweight.
l5      = 1.2           # 1.2  [m], height of arm axis above slide
l6      = 0.5           # 0.5  [m], size of cw box
m1      = 5.0           # 30   [kg], mass of counterweight
m2      = 0.085         # 0.08 [kg], mass of projectile
mb      = 1.5           # 1.5  [kg], mass of arm
# rotational inertia of the counterweight box [kg*m^2] ???

## we construct the arm in the 'loaded' position.
## the tip of l2 points low, and 
## the sling (l3) starts exactly horizontal, to eventually drop onto the slide
h1      = 0.2           # construction height of the sling

## cosanhyp
phi = math.acos((l5-h1)/l2)
a2=-l2*math.sin(phi)            # x-offset of the tip from the main axle
a1= l1*math.sin(phi*1.0)            # x-offset of the cw hinge from the main axle
b1= l1*math.cos(phi*1.0)            # y-offset of the cw hinge from the main axle
                                # positive y-axis points upwards.
# print phi, phi/math.pi*180

bg_color        = (255,255,255) # already done in run loop in pygame_framework.py
text_color      = (200,0,0)
trace_colors    = ( (.8,.4,.4), (.3,.1,.1), (.3,.1,.1), (.3,.1,.1), (.3,.1,.1) )
trace_colors    = ( (.5,.8,.8), (.8,1,1), (.8,1,1), (.8,1,1), (0.8,1,1) )

# file:///home/testy/src/python/pybox/doc/manual.htm#Box2D_v2.0.1_User_Manual
# /home/testy/src/svn-co/pybox2d/branches/box2d_2.0/doc/manual.htm#Filtering
PAINT_C         = 0x0001
PAINT_M         = 0x0000        # collides with nothing.
SHALLOW_C       = 0x0002
SHALLOW_M       = 0xFFFF-1-2    # collides neither with itself nor with PAINT
SOLID_C         = 0x0004        
SOLID_M         = 0xFFFF-1      # collides not with PAINT
BLOCK_C         = 0x0008        # counterweight box and its magical support.
BLOCK_M         = 0xFFFF-1-2-4  # the magical support only collides with the box.


## TODO:
# - remember the release point, vector, speed and distance across resets.
# - use the above collsion filters, 
# - implement a slide, so that the projectile does not touch the zeroplane in the beginning
# - add a contact hook, to freeze the projectle on impact, put a flag with counter and 
# - distance on it.
# - make slow motion switchable, timeStep in pygame_framework.py:Step()
# - implement a mechanic pin and sling, rather than a distance joint being destroyed at 45 deg.
# - implement a mechanic counterweight hinge, and add proper inertia to the counterweight.
# - add eye candy. Build a realistic looking catapult frame.


class Trebuchet (Framework):
    name="Trebuchet"     
    def __init__(self, full=1):
        self.togglePause = False
        self.doExit = False
        if not full:
            viewCenter = self._viewCenter
        super(Trebuchet, self).__init__()
        if full:
            self.setZoom(150.0)                         # 10 is default, pixels per unit
            self.setCenter((0, 300))                    # in screen units
            self.old_proj_trace = []
            self.dist_trace = []
            self.release_angle = 45
        else:
            self.setCenter(viewCenter)                  # __init__() just 'initialized' _viewCenter, restore it.
            self.old_proj_trace = self.proj_trace       # memorize one trace.

        self.proj_trace = []
        self.sling_trace = []
        self.shooting = False          # do not add to sling_trace yet.
        self.shotDistance = None
        self.singleStep = False

        sd=box2d.b2PolygonDef()          # ground plane is actually a box.
        sd.SetAsBox(120.0, 1.0, (116,0), 0)   # with, height will be twice these values, centered
        bd=box2d.b2BodyDef()
        bd.position = (0.0, -1.0)      # center point of the box. it extends from -20 to 180.
        zeroplane = self.world.CreateBody(bd)
        zeroplane.CreateShape(sd)

        ########### the throwing arm
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox((l1+l2+0.06)/2, (0.03)/2, ((l1+l2+0.06)/2-l1-0.06,0), 0)
        sd.density = 15                                 # approx, mb is recalculated in trigger
        sd.restitution = 0.5                            # bouncy, 0.5 is okay for wood
        bd=box2d.b2BodyDef() 
        bd.position = (0.0, l5)
        bd.angle = -math.pi*90/180-phi
        self.arm = self.world.CreateBody(bd) 
        self.arm.CreateShape(sd)
        sd=box2d.b2CircleDef() 
        sd.restitution = 0.5                            # bouncy, 0.5 is okay for wood
        sd.radius = 0.05                              # visualize main axis
        sd.density = 15
        self.arm.CreateShape(sd)
        sd.localPosition.Set(-l1,0)                    # visualize cw hinge
        self.arm.CreateShape(sd)
        sd.localPosition.Set(l2,0)                    # visualize pin
        sd.radius = 0.03
        self.arm.CreateShape(sd)
        # self.arm.SetMassFromShapes()               # not done, to start immobile. See K_t
        self.arm.SetMassFromShapes()               # not done, to start immobile. See K_t
        jd=box2d.b2RevoluteJointDef() 
        jd.Initialize(self.arm, zeroplane, (0,l5))
        jd.collideConnected = True
        jd.maxMotorTorque = 50.0           # (2.0 okay, 4.0 is too much) friction in the joint
        jd.motorSpeed = 0
        jd.enableMotor = True
        self.main_joint = self.world.CreateJoint(jd).getAsType() 

        ############ the projectile
        sd = box2d.b2CircleDef()
        sd.restitution = .3     # bounce the ball
        sd.radius = 0.05
        bd=box2d.b2BodyDef() 
        bd.position = (a2+l3, h1)
        bd.massData.mass = m2
        bd.massData.center = (0,0)
        bd.massData.I = 0.1       # guesstimate.
        bd.isBullet = True
        self.projectile = self.world.CreateBody(bd) 
        self.projectile.CreateShape(sd)
        jd=box2d.b2DistanceJointDef() 
        jd.Initialize(self.projectile, self.arm, (a2+l3, h1), (a2, h1) )
        jd.collideConnected = True
        self.sling = self.world.CreateJoint(jd).getAsType() 

        ############ the counterweight
        bd=box2d.b2BodyDef()
        bd.position = (a1, l5+b1-l4)
        cw = self.world.CreateBody(bd) 
        sd=box2d.b2PolygonDef() 
        sd.SetAsBox(l6/2,l6/3)
        sd.density = 100
        sd.restitution = 0
        cw.CreateShape(sd)
        # sd.SetAsBox(0.02,l6/2, (0, l6/2), 0)
        # cw.CreateShape(sd)            # requires collideConnected = False
        cw.SetMassFromShapes()
        print "cw mass = %g kg" % cw.GetMass()
        md = box2d.b2MassData()
        md.center = cw.GetLocalCenter()
        md.I = cw.GetInertia()*m1/cw.GetMass()
        md.mass = m1
        cw.massData = md           # now adjust the desired mass.
        print "cw mass = %g kg" % cw.GetMass()
        jd=box2d.b2RevoluteJointDef() 
        jd.Initialize(self.arm, cw, (a1,l5+b1))
        jd.collideConnected = True     # must be false here unless we have filtering.
        jd.maxMotorTorque = 1.0                     # friction in the joint
        jd.motorSpeed = 0
        jd.enableMotor = True
        self.world.CreateJoint(jd).getAsType() 


    def Keyboard(self, key):
        if key==K_r:
            self.__init__(full=0)
        if key==K_p:
            self.togglePause = True     # without True here, it starts paused.
        if key==K_q:
            self.DrawString(200,150, "**** Exiting ****")
            self.doExit = True
        if key==K_s:
            self.singleStep=True
            # FALLTHROUGH
        if (not self.shooting and (key==K_t or key==K_s)):
            # self.world.DestroyBody(self.support)
            self.arm.SetMassFromShapes()     # activate the hinge, calculate center of mass.
            self.main_joint.maxMotorTorque = 2.0           # (2.0 okay, 4.0 is too much) friction in the joint
            self.arm.WakeUp()
            self.shooting = True
            print "guess arm inertia = %g kg*m^2" % self.arm.GetInertia()
            md = box2d.b2MassData()
            md.center = self.arm.GetLocalCenter()
            md.I = self.arm.GetInertia()*mb/self.arm.GetMass()
            md.mass = mb
            self.arm.massData = md           # now adjust the desired mass.
            # assigning to massData.mass does not work. Sigh.
            print "calc arm inertia = %g kg*m^2" % self.arm.GetInertia()
            print "arm mass = %g kg" % self.arm.GetMass()
            # calculate the width/height ratio from release_angle
            self.release_ratio = 1 / math.tan(self.release_angle * math.pi / 180)

        if key==K_u or key==K_k:
            if self.release_angle < 85: self.release_angle += 5
        if key==K_d or key==K_j:
            if self.release_angle >  5: self.release_angle -= 5

    def Step(self, settings):
        # self.screen.fill(bg_color)
        # work around for Keyboard() not having access to settings.
        if self.singleStep:
            self.singleStep = False
            settings.pause = True
            settings.singleStep = True
        if self.togglePause:
            settings.pause = not settings.pause
            self.togglePause = False
        if self.shooting and self.sling:
            settings.slowMotion = 2
        self.DrawStringCR("T        trigger", text_color)
        self.DrawStringCR("S        single step", text_color)
        self.DrawStringCR("P        pause", text_color)
        self.DrawStringCR("R        reset", text_color)
        self.DrawStringCR("+        zoom in", text_color)
        self.DrawStringCR("-         zoom out", text_color)
        self.DrawStringCR("u        release earlier", text_color)
        self.DrawStringCR("d        release later", text_color)
        self.DrawStringCR("rmb   drag", text_color)
        self.DrawStringCR("F1      simulator menu", text_color)
        self.DrawStringCR("release_angle: (%g deg)" % self.release_angle)

        # settings.singleStep and settings.pause may be reset when we return from super.Step()
        do_trace = settings.singleStep or not settings.pause
        super(Trebuchet, self).Step(settings)
        c_idx = 0

        for p in self.dist_trace:
            self.debugDraw.DrawPoint(box2d.b2Vec2(p, 0), 8, (1,1,1))

        for p in self.old_proj_trace:
            if (c_idx == 0):
                self.debugDraw.DrawPoint(p, 3, (1,0.8,0.8))
            c_idx = (c_idx + 1) % 5

        for p in self.sling_trace:
            self.debugDraw.DrawSegment(p[0], p[1], trace_colors[c_idx])
            self.debugDraw.DrawPoint(p[0], 2, trace_colors[c_idx])
            self.debugDraw.DrawPoint(p[1], 1, trace_colors[c_idx])
            c_idx = (c_idx + 1) % 4

        for p in self.proj_trace:
            if (c_idx == 0):
                self.debugDraw.DrawPoint(p, 2, (0.6,0,0))
            c_idx = (c_idx + 1) % 4

        if self.shooting and not self.shotDistance:
            self.DrawStringCR("projectile: (%.1f, %.1f)" % 
              (self.projectile.position.x, self.projectile.position.y))
        if self.shotDistance:
            self.DrawStringCR("Shot distance: %.1f m" % self.shotDistance)
        if self.sling:
            sling_dx = self.sling.GetAnchor1().x - self.sling.GetAnchor2().x
            sling_dy = self.sling.GetAnchor1().y - self.sling.GetAnchor2().y
            if (self.shooting and do_trace):
              self.sling_trace.append((self.sling.GetAnchor1(), self.sling.GetAnchor2()))
            # when the sling points North North West, reaching 45 deg, we release the projectile
            if (sling_dx < 0 and sling_dy > 0 and hasattr(self, 'release_ratio')):
                # print "sling dx=%g dy=%g" % (sling_dx, sling_dy)
                if (sling_dy > -sling_dx * self.release_ratio):
                    self.world.DestroyJoint(self.sling)
                    self.sling = None
                    print "projectile released"
                    if settings.slowMotion > 2.0:
                      settings.slowMotion = 2.0
        else:
            p1 = self.world.GetGroundBody().GetWorldPoint(self.projectile.position)
            # HACK alert. This should depend on projectile collide with zeroplane.
            if (self.shooting and p1.y > 0.3):
              if (do_trace):
                self.proj_trace.append(p1)
            else:
              if (self.shooting):
                print "Distance: %g" % p1.x
                self.shotDistance = p1.x
                self.dist_trace.append(p1.x)
              self.shooting = False
        if self.doExit:
            exit()

if __name__=="__main__":
     main(Trebuchet)
