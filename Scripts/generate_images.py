from PIL import Image
import copy, sys

OUT_FILE_HEADER = '''#include "default_fbs.h"
#include <string.h>

#include "rgb_out.h"

'''

OUT_FILE_DATA = 'static const uint8_t _default{0}_fb[NDATA] = '

OUT_FILE_SWITCH_OPT = '''        case {0}:
            fb = _default{0}_fb;
            break;
'''

OUT_FILE_FUNC = '''
void get_default_fb(uint8_t index, uint8_t * buffer){{
    const uint8_t * fb;
    switch(index) {{
{0}        default:
            return;
    }}
    memcpy(buffer, fb, NDATA);
}}
'''

def load_img(filename):
  with open(filename, "rb") as f:
    img = Image.open(f)
    return [
      dict(zip(['r','g', 'b'], img.getdata().pixel_access()[(i,j)]))
        for j in xrange(img.size[1]) 
        for i in xrange(img.size[0])]

def plot(data):
  img = Image.new('RGB',(128,32))
  for r in range(32):
    for c in range(128):
      pixel = data[c + 128*r]
      img.getdata().pixel_access()[(c,r)] = (pixel['r'],pixel['g'],pixel['b'])
  img.show()
  return img

def new_enc(decoded):
  def trunc(pixel):
    return pixel & 240
  if len(decoded) != 32 * 128:
    raise Exception("Wrong image size")
  return reduce(lambda x,y: x+y,
    map(lambda x:[
      trunc(x[0]['r']) + (trunc(x[1]['r']) >> 4),
      trunc(x[0]['g']) + (trunc(x[1]['g']) >> 4),
      trunc(x[0]['b']) + (trunc(x[1]['b']) >> 4)
      ], zip(decoded[:16*128],decoded[16*128:])
    )
  )

def out(bufs, filename):
  with open(filename, "wb") as f:
    f.write(OUT_FILE_HEADER)
    switch_opt = ""
    for index,buf in enumerate(bufs):
      f.write(OUT_FILE_DATA.format(index))
      f.write("{\n")
      for i in range(16):
        f.write("  %s,\n"%(",".join("%d"%j for j in buf[384*i:384*(i+1)])))
      f.write("};\n")
      switch_opt+=OUT_FILE_SWITCH_OPT.format(index)
    f.write(OUT_FILE_FUNC.format(switch_opt))

def fade_lateral(data):
  data = copy.deepcopy(data)
  level = -1
  direction = 1
  for i,d in enumerate(data):
    if i%4 == 0:
      level += direction
      if level == -1:
        direction = 1
        level = 0
      if level == 16:
        direction = -1
        level = 15
    pixel = level << 4
    d['r'] = d['r']/255 * pixel
    d['g'] = d['g']/255 * pixel
    d['b'] = d['b']/255 * pixel
  return data

if __name__ == '__main__':
  data0 = load_img("Data/default_inc.bmp")
  data1 = fade_lateral(data0)
  plot(data0)
  plot(data1)
  out([new_enc(data0), new_enc(data1)], sys.argv[1])