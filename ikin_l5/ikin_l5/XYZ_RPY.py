import os
import yaml
from ament_index_python.packages import get_package_share_directory

import pprint

class XYZ_RPY:
  def __init__(self):
    pp = pprint.PrettyPrinter()

    self.dhv = readDH()
    self.links = readLinks()
    self.xyz_rpy = {}

    self.dhv.pop('fixed_joints')

    self.create_xyz_rpy()
    pp.pprint(self.xyz_rpy)
    pp.pprint(self.links)
    self.save()

  def create_base_el1(self):
    self.xyz_rpy['base-el1'] = {}
    self.xyz_rpy['base-el1']['x'] = 0
    self.xyz_rpy['base-el1']['y'] = 0
    self.xyz_rpy['base-el1']['z'] = self.links['base']['w']
    self.xyz_rpy['base-el1']['roll'] = 0
    self.xyz_rpy['base-el1']['pitch'] = 0
    self.xyz_rpy['base-el1']['yaw'] = 0
    
  def crate_el1_el2(self):
    self.xyz_rpy['el1-el2'] = {}
    self.xyz_rpy['el1-el2']['x'] = self.links['el1']['a']
    self.xyz_rpy['el1-el2']['y'] = 0
    self.xyz_rpy['el1-el2']['z'] = self.links['el1']['l'] + self.dhv['el1-el2']['d']
    self.xyz_rpy['el1-el2']['roll'] = self.links['el1']['alpha']
    self.xyz_rpy['el1-el2']['pitch'] = 0
    self.xyz_rpy['el1-el2']['yaw'] = self.dhv['el1-el2']['t']

  def create_el2_el3(self):
    self.xyz_rpy['el2-el3'] = {}
    self.xyz_rpy['el2-el3']['x'] = self.links['el2']['a']
    self.xyz_rpy['el2-el3']['y'] = 0
    self.xyz_rpy['el2-el3']['z'] = self.links['el2']['r'] + self.dhv['el2-el3']['d']
    self.xyz_rpy['el2-el3']['roll'] = self.links['el2']['alpha']
    self.xyz_rpy['el2-el3']['pitch'] = 0
    self.xyz_rpy['el2-el3']['yaw'] = self.dhv['el2-el3']['t']

  def create_el3_tool(self):
    self.xyz_rpy['el3-tool'] = {}
    self.xyz_rpy['el3-tool']['x'] = self.links['el3']['a']
    self.xyz_rpy['el3-tool']['y'] = 0
    self.xyz_rpy['el3-tool']['z'] = self.dhv['el3-tool']['d']
    self.xyz_rpy['el3-tool']['roll'] = self.links['el3']['alpha']
    self.xyz_rpy['el3-tool']['pitch'] = 0
    self.xyz_rpy['el3-tool']['yaw'] = self.dhv['el3-tool']['t']

  def create_xyz_rpy(self):
    self.create_base_el1()
    self.crate_el1_el2()
    self.create_el2_el3()
    self.create_el3_tool()

  def save(self):
    with open(os.path.join(get_package_share_directory('ikin_l5'), 'xyz_rpy.yaml'), 'w') as file:
        yaml.dump(self.xyz_rpy, file)

def readDH():
  with open(os.path.join(get_package_share_directory('ikin_l5'), 'joints.yaml'), 'r') as file:
    dhv = yaml.load(file, Loader=yaml.FullLoader)

  return dhv

def readLinks():
  with open(os.path.join(get_package_share_directory('ikin_l5'), 'links.yaml'), 'r') as file:
    links = yaml.load(file, Loader=yaml.FullLoader)

  return links

def main():
  xyz_rpy = XYZ_RPY()


if __name__=='__main__':
  main()
