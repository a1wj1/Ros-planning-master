#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @File     : test.py
# @Project  : Hermes
# @Software : PyCharm
# @Author   : 魏浩源
# @Email    : weihaoyuan2@126.com
# @Time     : 2020/7/4 上午10:31

import json
if __name__ == '__main__':
    with open('./config.json', 'r', encoding='utf8')as fp:
        json_data = json.load(fp)
    print(json_data['gpsReceiveConfig']['gpsPort'])