#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @File     : map.py
# @Project  : Sleipnir
# @Software : PyCharm
# @Author   : why
# @Email    : weihaoyuan2@126.com
# @Time     : 2020/7/10 下午3:49

import cv2
import tools.WGS84 as wg
import numpy as np
import pandas as pd
from math import *
from tools.path import *

class Map():
    def __init__(self):
        self.displayMapWidth=910
        self.displayMapHeight=340
        self.trajectoryPoint=[]
        self.zoomStep=0
        self.zoomRate=1

        self.centerX=10000
        self.centerY=10000
        self.centerGPSX=0
        self.centerGPSY=0
        self.movePixel=(0,0)

        self.leftTopLat=23.072151310102857
        self.leftTopLon=113.358993530273438
        self.rightBottomLat=23.032346928548652
        self.rightBottomLon=113.423538208007812
        self.deviationX=540.4385031
        self.deviationY=-230.6385366
        self.leftTopX,self.leftTopY = wg.CoordConverter.LatLonToUTMXY(self.leftTopLat,self.leftTopLon)
        self.rightBottomX,self.rightBottomY = wg.CoordConverter.LatLonToUTMXY(self.rightBottomLat,self.rightBottomLon)

        self.leftTopX-=self.deviationX
        self.leftTopY-=self.deviationY
        self.rightBottomY-=self.deviationY
        self.rightBottomX-=self.deviationX

        self.Width=abs(self.rightBottomX-self.leftTopX)
        self.Height=abs(self.rightBottomY-self.leftTopY)

        self.mapImage=cv2.imread(tools_path+'/map/UniversityTown.tif')
        self.mapImageBackUp=self.mapImage.copy()
        self.mapImageSize=self.mapImage.shape

        self.rateWidth=self.mapImageSize[1]/self.Width
        self.rateHeight=self.mapImageSize[0]/self.Height

        self.gpsPointUTM=[]
        self.drawGPSColor=((255,0,0),(255,255,),(0,255,0),(0,255,255),(255,0,255),(0,0,255),(0,0,0))
        self.drawGPSColorID=0

        self.leftBoundary=0
        self.rightBoundary=0
        self.topBoundary=0
        self.bottomBoundary=0

        self.leftX=0
        self.topY=0
        self.rightX=0
        self.bottomY=0

        self.carImage=cv2.imread(tools_path+'/map/carmap.png')
        self.carImageSize=(100,100)
        self.carImage=cv2.resize(self.carImage,self.carImageSize)
        self.carImage=cv2.transpose(self.carImage)
        self.carHeight, self.carWidth =self.carImage.shape[:2]
        self.carImageRotation=np.zeros(self.carImageSize)
        self.carHistoryArea=None
        self.roiCarImage=None

    def limCenter(self):
        if self.centerX<self.zoomRate*self.displayMapWidth/2:
            self.centerX=int(self.zoomRate*self.displayMapWidth/2)
        if self.centerX>self.mapImageSize[1]-(self.zoomRate*self.displayMapWidth/2):
            self.centerX=int(self.mapImageSize[1]-(self.zoomRate*self.displayMapWidth/2))
        if self.centerY<self.zoomRate*self.displayMapHeight/2:
            self.centerY=int(self.zoomRate*self.displayMapHeight/2)
        if self.centerY>self.mapImageSize[0]-(self.zoomRate*self.displayMapHeight/2):
            self.centerY=int(self.mapImageSize[0]-(self.zoomRate*self.displayMapHeight/2))

    def setCenter(self,center):
        centerPixel=self.UTMtoPixel(center[0],center[1])
        self.centerX=centerPixel[0]
        self.centerY=centerPixel[1]

    def getDisplaySize(self):
        return (self.displayMapWidth,self.displayMapHeight)

    def setZoomRate(self,rate):
        self.zoomRate=rate

    def move(self,moveDis):
        if moveDis != self.movePixel:
            self.centerX-=(moveDis[0]-self.movePixel[0])*self.zoomRate
            self.centerY-=(moveDis[1]-self.movePixel[1])*self.zoomRate
            self.limCenter()
            # print('movePixel:', self.movePixel,'\n','data:',data)
            # print('center',self.centerX,self.centerY)
            self.movePixel=moveDis

    def setMovePixel(self,movePix):
        self.movePixel=movePix
        #return (self.centerX+data[0]*self.zoomRate,self.centerY+data[1]*self.zoomRate)

    # if cmd=='setTrajectoryPoint' and data is not None:
    #     self.trajectoryPoint=data
    #     print('setTrajectoryPoint')

    def drawTrajectory(self,traj):
        for index,row in traj.iterrows():
            pixel = self.UTMtoPixel(row.x, row.y)
            # relativeX = pixel[0]-self.leftTopX
            # relativeY = abs(pixel[1]-self.leftTopY)
            # print('drawPoint:',relativeX,relativeY)
            # cv2.circle(image,(relativeX,relativeY),1,(0,0,255),-1)
            cv2.circle(self.mapImage, (pixel[0], pixel[1]), 2, self.drawGPSColor[self.drawGPSColorID], -1)
        self.drawGPSColorID += 1
        val = 0
        for i in self.drawGPSColor[self.drawGPSColorID]:
            val += i
        if val == 0:
            self.drawGPSColorID = 0
        # pixel = self.UTMtoPixel(self.gpsPointUTM[1000][0], self.gpsPointUTM[1000][1]+100)
        # relativeX = pixel[0] - self.leftX
        # relativeY = pixel[1] - self.topY + 1500
        # cv2.circle(imageShow, (relativeX,relativeY), 50, (255, 0, 0), 4)
        print('drawTrajectory')

    def UTMtoPixel(self,X,Y):
        relativeX=X-self.leftTopX
        relativeY=Y-self.leftTopY

        pixelX=abs(int(relativeX*self.rateWidth))
        pixelY=abs(int(relativeY*self.rateHeight))

        return (pixelX,pixelY)

    def getDisplayMap(self,type='pixel'):
        if type=='UTM':
            pixel=self.UTMtoPixel(self.centerX,self.centerY)
            leftX=pixel[0]-self.displayMapWidth/2
            rightX=pixel[0]+self.displayMapWidth/2
            topY=pixel[1]-self.displayMapHeight/2
            bottomY=pixel[1]+self.displayMapHeight/2
            return cv2.resize(self.mapImage[topY:bottomY, leftX:rightX, :],(self.displayMapWidth, self.displayMapHeight))
        elif type=='pixel':
            leftX = self.centerX - int(self.displayMapWidth * self.zoomRate / 2)
            rightX = self.centerX + int(self.displayMapWidth * self.zoomRate / 2)
            topY = self.centerY - int(self.displayMapHeight * self.zoomRate / 2)
            bottomY = self.centerY + int(self.displayMapHeight * self.zoomRate / 2)
            if leftX<0:
                rightX +=abs(leftX)
                leftX = 0
            if rightX>self.mapImageSize[1]:
                leftX-=rightX-self.mapImageSize[1]
                rightX=self.mapImageSize[1]
            if topY<0:
                bottomY+=abs(topY)
                topY = 0
            if bottomY>self.mapImageSize[0]:
                topY-=bottomY-self.mapImageSize[0]
                bottomY = self.mapImageSize[0]
            return cv2.resize(self.mapImage[topY:bottomY, leftX:rightX, :],(self.displayMapWidth, self.displayMapHeight))
        # leftX=int(leftX*zoomRate)
        # rightX=int(rightX*zoomRate)
        # topY=int(topY*zoomRate)
        # bottomY=int(bottomY*zoomRate)
        #return leftX,rightX,topY,bottomY

    def drawCar(self,center,rotateDegree=0):

        # 旋转后的尺寸
        heightNew = int(self.carWidth * fabs(sin(radians(rotateDegree))) + self.carHeight * fabs(cos(radians(rotateDegree))))
        widthNew = int(self.carHeight * fabs(sin(radians(rotateDegree))) + self.carWidth * fabs(cos(radians(rotateDegree))))

        matRotation = cv2.getRotationMatrix2D((self.carWidth / 2, self.carHeight / 2), rotateDegree, 1)

        matRotation[0, 2] += (widthNew - self.carWidth) / 2
        matRotation[1, 2] += (heightNew - self.carHeight) / 2

        self.carImageRotation = cv2.warpAffine(self.carImage, matRotation, (widthNew, heightNew), borderValue=(0, 0, 0))
        carImageRotationWidth,carImageRorationHeight=self.carImageRotation.shape[:2]
        centerPixel=self.UTMtoPixel(center[1],center[0])
        #centerPixel=center
        X1=int(centerPixel[0]-carImageRotationWidth/2)
        X2=int(centerPixel[0]+carImageRotationWidth/2)
        Y1=int(centerPixel[1]-carImageRorationHeight/2)
        Y2=int(centerPixel[1]+carImageRorationHeight/2)
        if self.roiCarImage is not None:
            roiHistoryAreaWidth,roiHistoryAreaHeight=self.carHistoryArea.shape[:2]
            for i1 in range(roiHistoryAreaWidth):
                for j1 in range(roiHistoryAreaHeight):
                    self.roiCarImage[j1][i1]=self.carHistoryArea[j1][i1]
        self.roiCarImage=self.mapImage[Y1:Y2,X1:X2,:]
        self.carHistoryArea=self.roiCarImage.copy()
        # cv2.imshow('caeImage',self.carImageRotation)
        # cv2.waitKey(0)
        try:
            for i in range(carImageRotationWidth):
                for j in range(carImageRorationHeight):
                    if self.carImageRotation[j][i][0]+self.carImageRotation[j][i][1]+self.carImageRotation[j][i][2] != 0:
                        # print('roiCarImage:',self.roiCarImage[j][i],'carImageRotation:',self.carImageRotation[j][i],'i,j:',i,j)
                        # print('roiCarImage.shape:',self.roiCarImage.shape,'carImageRotation.shape:',self.carImageRotation.shape)
                        # print('roiCarImageWidth:',carImageRotationWidth,'roiCarImageHeight',carImageRorationHeight)
                        self.roiCarImage[j][i]=self.carImageRotation[j][i]
        except:
            print('ERROR*****')

    def clearTrajectory(self):
        self.mapImage=self.mapImageBackUp.copy()

if __name__ == '__main__':
    map=Map()

