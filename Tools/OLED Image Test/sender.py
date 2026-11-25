import sys,time,serial
from PIL import Image,ImageOps
im=ImageOps.exif_transpose(Image.open(sys.argv[2])).convert('L')
im=ImageOps.contain(im,(128,64))
canvas=Image.new('L',(128,64));canvas.paste(im,((128-im.width)//2,(64-im.height)//2))
mono=canvas.convert('1');mono.resize((512,256),Image.Resampling.NEAREST).save('preview.png')
with serial.Serial(sys.argv[1],115200,timeout=6) as s:
 time.sleep(3);s.reset_input_buffer();s.write(b'OLED'+bytes([64])+mono.tobytes());s.flush();r=s.readline().decode(errors='replace').strip();print('ESP32 response:',r);sys.exit(0 if r=='DISPLAYED' else 1)
