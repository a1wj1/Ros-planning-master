#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @File     : myGPX.py
# @Project  : Sleipnir
# @Software : PyCharm
# @Author   : why
# @Email    : weihaoyuan2@126.com
# @Time     : 2020/8/12 下午7:26
from lxml import etree
import time
import lxml
from sys import stdout

def _typed_property(name,expected_type,minimum=float('-inf'),maximum=float('inf')):
    storage_name='_'+name
    @property
    def prop(self):
        value=getattr(self,storage_name)
        if value is None:
            return False
        else:
            return value
    @prop.setter
    def prop(self,value):
        if expected_type=='numeral':
            if not isinstance(value, int) and not isinstance(value,float) and not isinstance(value, type(None)) :
                raise TypeError('{}must be a {}'.format(name, 'number'))
        elif not isinstance(value,expected_type) and not isinstance(value,type(None)):
            raise TypeError('{}must be a {}'.format(name,expected_type))
        if isinstance(value,int) or isinstance(value,float):
            if value<minimum:
                raise ValueError('{} must be greater than {}'.format(name,minimum))
            elif value>maximum:
                raise ValueError('{} must be less than {}'.format(name,maximum))
        setattr(self,storage_name,value)
    return prop

def get_date_time():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.localtime())

class __Iter():
    def __init__(self,index_number):
        self.index_number=index_number
        self.index=0
    def __iter__(self):
        return self
    def __next__(self):
        if self.index==self.index_number:
            self.index=0
            raise StopIteration
        else:
            self.index+=1
            attribute=list(self.__dict__.keys())[self.index-1]
            if attribute[0] =='_':
                attribute_value=eval('self.{}'.format(attribute[1:]))
                return attribute[1:],attribute_value
            else:
                attribute_value = eval('self.{}'.format(attribute))
                return attribute, attribute_value

class Link(__Iter):
    link=_typed_property('link',str)
    link_text=_typed_property('link_text',str)
    link_type=_typed_property('link_type',str)
    def __init__(self):
        self.link=None
        self.link_text=None
        self.link_type=None
        super(Link, self).__init__(len(vars(self)))

class Person(__Iter):
    name=_typed_property('name',str)
    email=_typed_property('email',str)
    def __init__(self):
        self.name=None
        self.email=None
        self.link=Link()
        super(Person, self).__init__(len(vars(self)))

class Copyright(__Iter):
    author=_typed_property('author',str)
    license=_typed_property('license',str)
    year=_typed_property('year',int,minimum=0,maximum=9999)
    def __init__(self):
        self.author=None
        self.year=None
        self.license=None
        super(Copyright, self).__init__(len(vars(self)))

class Bounds():
    def __init__(self):
        self._minlat=None
        self._maxlat=None
        self._minlon=None
        self._maxlon=None
        self._exist_bounds=False

    def get_bounds(self):
        return {'minlat':self._minlat,'maxlat':self._maxlat,'minlon':self._minlon,'maxlon':self._maxlon}

    def set_bounds(self,minlat,maxlat,minlon,maxlon):
        if max(minlat,maxlat)<=90 and min(minlat,maxlat)>=-90:
            self._minlat=str(minlat)
            self._maxlat=str(maxlat)
        else:
            raise ValueError('Latitude should be between -90 and +90')
        if max(minlon,maxlon)<=180 and min(minlon,maxlon)>=-180:
            self._minlon=str(minlon)
            self._maxlon=str(maxlon)
        else:
            raise ValueError('Longitude should be between -180 and +180')
        self._exist_bounds=True

    @property
    def exist(self):
        return self._exist_bounds

class Extensions(__Iter):
    def __init__(self,**kwargs):
        for key,val in kwargs.items():
            exec('self.{}=str({})'.format(key,str(val)))
        super(Extensions, self).__init__(len(vars(self)))

class Metadata(__Iter):
    name=_typed_property('name',str)
    desc=_typed_property('desc',str)
    time=_typed_property('time',str)
    keywords=_typed_property('keywords',str)
    extensions=_typed_property('extensions',Extensions)

    def __init__(self):
        self.name=None
        self.desc=None
        self.author=Person()
        self.copyright=Copyright()
        self.link=Link()
        self.time=None
        self.keywords=None
        self.bounds=Bounds()
        self.extensions=None
        super(Metadata,self).__init__(len(vars(self)))

    def set_time(self):
        self.time=get_date_time()

class Wpt(__Iter):
    lon=_typed_property('lon','numeral',minimum=-180.0,maximum=180.0)
    lat=_typed_property('lat','numeral',minimum=-90.0,maximum=90.0)
    ele=_typed_property('ele',str)
    time = _typed_property('time', str)
    magvar=_typed_property('magvar','numeral',minimum=0.0,maximum=360.0)
    geoidheight=_typed_property('geoidheight','numeral')
    name=_typed_property('name',str)
    cmt=_typed_property('cmt',str)
    desc=_typed_property('desc',str)
    src=_typed_property('src',str)
    sym=_typed_property('sym',str)
    type=_typed_property('type',str)
    fix=_typed_property('fix',str)
    sat=_typed_property('sat',int,minimum=0)
    hdop=_typed_property('hdop','numeral')
    vdop=_typed_property('vdop','numeral')
    pdop=_typed_property('pdop','numeral')
    ageofdgpsdata=_typed_property('ageofdgpsdata','numeral')
    dgpsid=_typed_property('dgpsid',int,minimum=0,maximum=1023)
    extensions=_typed_property('extensions',Extensions)
    def __init__(self):
        self.lat=None
        self.lon=None
        self.ele=None
        self.time=None
        self.magvar=None
        self.geoidheight=None
        self.name=None
        self.cmt=None
        self.desc=None
        self.src=None
        self.link=Link()
        self.sym=None
        self.type=None
        self.fix=None
        self.sat=None
        self.hdop=None
        self.vdop=None
        self.pdop=None
        self.ageofdgpsdata=None
        self.dgpsid=None
        self.extensions=None
        super(Wpt, self).__init__(len(vars(self)))

    def set_time(self):
        self.time=get_date_time()

class Rte(__Iter):
    name=_typed_property('name',str)
    cmt=_typed_property('cmt',str)
    desc=_typed_property('desc',str)
    src=_typed_property('src',str)
    number=_typed_property('number',int,minimum=0)
    type=_typed_property('type',str)
    extensions=_typed_property('extensions',Extensions)

    def __init__(self):
        self.name=None
        self.cmt=None
        self.desc=None
        self.src=None
        self.link=Link()
        self.number=None
        self.type=None
        self.extensions=None
        super(Rte, self).__init__(len(vars(self)))

class Trkseg():
    extensions=_typed_property('extensions',Extensions)
    def __init__(self):
        self.extensions=None

class Trk(__Iter):
    name=_typed_property('name',str)
    cmt=_typed_property('cmt',str)
    desc=_typed_property('desc',str)
    src=_typed_property('src',str)
    number=_typed_property('number',int,minimum=0)
    type=_typed_property('type',str)
    extensions=_typed_property('extensions',Extensions)
    def __init__(self):
        self.name=None
        self.cmt=None
        self.desc=None
        self.src=None
        self.link=Link()
        self.number=None
        self.type=None
        self.extensions=None
        self.trkseg=Trkseg()
        super(Trk, self).__init__(len(vars(self)))

def add_extensions(element,element_name,extensions):
    extensions_element=etree.SubElement(element,element_name)
    for child_element_name,value in extensions:
        child_element=etree.SubElement(extensions_element,child_element_name)
        child_element.text=str(value)

def add_link(element,element_name,link):
    if link.link:
        link_element = etree.SubElement(element, element_name, href=link.link)
        for child_element_name,value in link:
            if value:
                child_element=etree.SubElement(link_element,child_element_name)
                child_element.text=str(value)

def add_email(element,email):
    if not email:
        id, domain = email.split('@')
        email_element = etree.SubElement(element, 'email', id=id, domain=domain)

def add_person(element,element_name,person):
    if person.name:
        person_element=etree.SubElement(element,element_name)
        element_name=etree.SubElement(person_element,'name')
        element_name.text=person.name
        add_email(person_element,person.email)
        add_link(person_element,'link',person.link)

def add_copyright(element, element_name,copyright):
    if copyright.author:
        copyright_element=etree.SubElement(element,element_name,author=copyright.author)
        for child_element_name,value in copyright:
            if value:
                child_element=etree.SubElement(copyright_element,child_element_name)
                child_element.text=str(value)

def add_trkseg(element,element_name,trkseg):
    trkseg_element=etree.SubElement(element,element_name)
    if trkseg.extensions:
        add_extensions(trkseg_element,'extensions',trkseg.extensions)
    return trkseg_element

def add_metadata(element, element_name,metadata ):
    metadata_element = etree.SubElement(element, element_name)
    for child_element_name,value in metadata:
        if not value:
            continue
        elif isinstance(value,Person):
            add_person(metadata_element,'author',value)
        elif isinstance(value,Copyright) and value.author:
            add_copyright(metadata_element,'copyright',value)
        elif isinstance(value,Link):
            add_link(metadata_element,'link',value)
        elif isinstance(value,Bounds):
            if value.exist:
                bounds_element = etree.SubElement(metadata_element, 'bounds')
                Dict = value.get_bounds()
                for key, val in Dict.items():
                    bounds_element.set(key, val)
        elif isinstance(value,Extensions):
            add_extensions(metadata_element,'extensions',value)
        elif value:
            child_element=etree.SubElement(metadata_element,child_element_name)
            child_element.text=str(value)
    return metadata_element

def add_wpt(element,element_name,wpt):
    wpt_element=etree.SubElement(element,element_name,lat=str(wpt.lat),lon=str(wpt.lon))
    for child_element_name,value in wpt:
        if child_element_name == 'lat' or child_element_name == 'lon':
            continue
        elif isinstance(value,Link):
            add_link(wpt_element,child_element_name,value)
        elif isinstance(value,Extensions):
            add_extensions(wpt_element,child_element_name,value)
        elif value:
            child_element=etree.SubElement(wpt_element,child_element_name)
            child_element.text=str(value)
    return wpt_element

def add_rte(element,element_name,rte):
    rte_element=etree.SubElement(element,element_name)
    for child_element_name,value in rte:
        if isinstance(value,Link):
            add_link(rte_element,child_element_name,value)
        elif isinstance(value,Extensions):
            add_extensions(rte_element,child_element_name,value)
        elif value:
            child_element=etree.SubElement(rte_element,child_element_name)
            child_element.text=str(value)
    return rte_element

def add_trk(element,element_name,trk):
    trk_element=etree.SubElement(element,element_name)
    for child_element_name,value in trk:
        if isinstance(value,Link):
            add_link(trk_element,child_element_name,value)
        elif isinstance(value,Extensions):
            add_extensions(trk_element,child_element_name,value)
        elif value:
            child_element=etree.SubElement(trk_element,child_element_name)
            child_element.text=str(value)
    trkseg_element=add_trkseg(trk_element,'trkseg',trk.trkseg)
    return [trk_element,trkseg_element]

class Gpx():
    def __init__(self):
        self.gpx_element=etree.Element('gpx',versin='1.1')
        self.metadata=Metadata()
        self.metadata.set_time()
        self.metadata_element=None
        self.waypoint_dict={}
        self.rte_dict={}
        self.rte_point_name_dict={}
        self.trk_dict={}

    def update_metadata(self):
        if self.metadata_element is None:
            self.metadata_element=add_metadata(self.gpx_element,'metadata',self.metadata)
        else:
            self.gpx_element.remove(self.metadata_element)
            self.metadata_element=add_metadata(self.gpx_element,'metadata',self.metadata)

    def remove_metadata(self):
        self.gpx_element.remove(self.metadata_element)

    def add_waypoint(self,wpt):
        if not isinstance(wpt,Wpt):
            raise TypeError('wpt must be a '+str(type(Wpt())))
        if not wpt.name and wpt.lat and wpt.lon:
            raise ValueError('wptpoint must specify longitude, latitude and name')
        if not wpt.time:
            wpt.set_time()
        if wpt.name in self.waypoint_dict:
            raise ValueError(wpt.name +' already exist')
        self.waypoint_dict[wpt.name]=add_wpt(self.gpx_element,'wpt',wpt)
        
    def remove_waypoint(self,waypoint_name):
        self.gpx_element.remove(self.waypoint_dict[waypoint_name])
        self.waypoint_dict.pop(waypoint_name)

    def __update_wpt(self,waypoint_element,wpt):
        child_element_list = waypoint_element.xpath('./*')
        child_element_dict = {}
        for child_elment in child_element_list:
            child_element_dict[child_elment.tag] = child_elment
        for child_element_name, value in wpt:
            if isinstance(value, Link):
                if len(waypoint_element.xpath('./link')):
                    waypoint_element.remove(waypoint_element.xpath('./link')[0])
                add_link(waypoint_element, child_element_name, value)
            elif isinstance(value, Extensions):
                if len(waypoint_element.xpath('./extensions')):
                    waypoint_element.remove(waypoint_element.xpath('./extensions')[0])
                add_extensions(waypoint_element, child_element_name, value)

            elif value is not False:
                if child_element_name in list(child_element_dict.keys()):
                    child_element_dict[child_element_name].text = value
                elif child_element_name == 'lat':
                    waypoint_element.set('lat', str(wpt.lat))
                elif child_element_name == 'lon':
                    waypoint_element.set('lon', str(wpt.lon))
                else:
                    child_element = etree.SubElement(waypoint_element, child_element_name)
                    child_element.text = str(value)

    def update_waypoint(self,waypoint_name,wpt):
        if not isinstance(wpt,Wpt):
            raise TypeError('wpt must be a '+str(type(Wpt)))
        self.__update_wpt(self.waypoint_dict[waypoint_name],wpt)
        if wpt.name:
            self.waypoint_dict[wpt.name]=self.waypoint_dict.pop(waypoint_name)

    def add_rte(self,rte):
        if not isinstance(rte,Rte):
            raise TypeError('rte must be a '+str(type(Rte)))
        if not rte.name:
            raise ValueError('rte must specify name')
        if rte.name in self.rte_dict:
            raise ValueError(rte.name +' already exist')
        self.rte_dict[rte.name]=add_rte(self.gpx_element,'rte',rte)
        self.rte_point_name_dict[rte.name]=[]

    def add_rte_point(self,rte_name,wpt):
        if not isinstance(wpt, Wpt):
            raise TypeError('wpt must be a ' + str(type(Wpt)))
        if not wpt.name and wpt.lat and wpt.lon:
            raise ValueError('wptpoint must specify longitude, latitude and name')
        if not wpt.time:
            wpt.set_time()
        if wpt.name in self.rte_point_name_dict[rte_name]:
            raise ValueError(wpt.name +' already exist in '+rte_name)
        self.rte_point_name_dict[rte_name].append(wpt.name)
        add_wpt(self.rte_dict[rte_name], 'rtept', wpt)

    def remove_rte(self,rte_name):
        self.gpx_element.remove(rte_name)
        self.rte_dict.pop(rte_name)

    def remove_rte_watpoint(self,rte_name,wpt_name):
        rtept_element_list=self.rte_dict[rte_name].xpath('./rtept')
        for wpt_element in rtept_element_list:
            if wpt_element.xpath('/.name')[0].xpath('text()')[0] == wpt_name:
                self.rte_dict[rte_name].remove(wpt_element)

    def update_rte(self,rte_name,rte):
        if not isinstance(rte,Rte):
            raise TypeError('rte must be a '+str(type(Rte)))
        child_element_list = self.rte_dict[rte_name].xpath('./*')
        child_element_dict = {}
        for child_elment in child_element_list:
            child_element_dict[child_elment.tag] = child_elment
        for child_element_name, value in rte:
            if isinstance(value, Link):
                if len(self.rte_dict[rte_name].xpath('./link')):
                    self.rte_dict[rte_name].remove(self.rte_dict[rte_name].xpath('./link')[0])
                add_link(self.rte_dict[rte_name], child_element_name, value)
            elif isinstance(value, Extensions):
                if len(self.rte_dict[rte_name].xpath('./extensions')):
                    self.rte_dict[rte_name].remove(self.rte_dict[rte_name].xpath('./extensions')[0])
                add_extensions(self.rte_dict[rte_name], child_element_name, value)
            elif value is not False:
                if child_element_name in list(child_element_dict.keys()):
                    child_element_dict[child_element_name].text = value
                else:
                    child_element = etree.SubElement(self.rte_dict[rte_name], child_element_name)
                    child_element.text = str(value)
        if rte.name:
            self.rte_dict[rte.name]=self.rte_dict.pop(rte_name)
            self.rte_point_name_dict[rte_name]=self.rte_point_name_dict.pop(rte_name)

    def update_rte_waypoint(self,rte_name,wpt_name,wpt):
        if not isinstance(wpt,Wpt):
            raise TypeError('wpt must be a '+str(type(Wpt)))
        rtept_element_list = self.rte_dict[rte_name].xpath('./rtept')
        for wpt_element in rtept_element_list:
            if wpt_element.xpath('./name')[0].xpath('text()')[0] == wpt_name:
                self.__update_wpt(wpt_element,wpt)

    def add_track(self,trk):
        if not isinstance(trk,Trk):
            raise TypeError('trk must be a '+str(type(Trk)))
        if not trk.name:
            raise ValueError('trk must specify name')
        if trk.name in self.trk_dict:
            raise ValueError(trk.name+' already exist')
        self.trk_dict[trk.name]=add_trk(self.gpx_element,'trk',trk)

    def remove_track(self,trk_name):
        self.gpx_element.remove(trk_name)
        self.trk_dict.pop(trk_name)

    def update_track(self,trk_name,trk):
        if not isinstance(trk,Trk):
            raise TypeError('trk must be a '+str(type(Trk)))
        child_element_list = self.trk_dict[trk_name][0].xpath('./*')
        child_element_dict = {}
        for child_elment in child_element_list:
            child_element_dict[child_elment.tag] = child_elment
        for child_element_name, value in trk:
            if isinstance(value, Link):
                add_link(self.trk_dict[trk_name][0], child_element_name, value)
            elif isinstance(value, Extensions):
                add_extensions(self.trk_dict[trk_name][0], child_element_name, value)
            elif value is not False:
                if child_element_name in list(child_element_dict.keys()):
                    child_element_dict[child_element_name].text = value
                else:
                    child_element = etree.SubElement(self.trk_dict[trk_name][0], child_element_name)
                    child_element.text = str(value)


    def print_xml(self):
        print(etree.tostring(self.gpx_element, pretty_print=True).decode('utf-8'))


if __name__ == '__main__':
    import sys
    root_element = etree.Element('gpx', versin='1.1')
    # for key,val in {'a':'1','b':'2'}.items():
    #     element=etree.SubElement(root_element,key)
    #     element.text=val
    #addPerson(root_element,'why','wwhy','weihaoyuan2@126.com','www.baidu.com','www','wwww')
    m=Metadata()
    #m.__getattribute__()
    m.name='weihaoyuan'
    m.desc='xipu'
    m.author.name='weihaoyuan'
    m.author.email='weihaoyuan@126.com'
    m.author.link.link='www.baidu.com'
    m.author.link.link_text='baidu.com'
    m.author.link.link_type='image'
    m.copyright.author='weihaoyuan'
    m.copyright.year=2020
    m.copyright.license='BSD'
    m.link.link='www.baidu.com'
    m.link.link_text='baidu.com'
    m.link.link_type='image'
    m.set_time()
    m.keywords='456'
    m.bounds.set_bounds(0,0,0,0)
    ex=Extensions(a='1',b='2')
    m.extensions=ex
    add_metadata(root_element,'Metadata',m)
    wpt=Wpt()
    wpt.lat=10
    wpt.lon=20
    wpt_element=add_wpt(root_element,'wpt',wpt)
    print(etree.tostring(root_element,pretty_print=True).decode('utf-8'))
    wpt_element.set('lon','666')
    print(etree.tostring(root_element, pretty_print=True).decode('utf-8'))