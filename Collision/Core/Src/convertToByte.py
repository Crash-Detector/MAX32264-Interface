import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)

byte_arr = np.fromfile( "algorithm.msbl", dtype=np.uint8)

f_w = open( "algo_as_bytes_1.txt", 'w' )

print( byte_arr.size )
#print( byte_arr )
#print( byte_arr[ 0 ] )

byteOf = [ ]

for byte in byte_arr:
    #print (byte)
    #print (type(byte))
    #byteOf.append( byte )
    write_byte = str(byte) + ", " #0x" + chr(byteOf[0]) + chr(byteOf[1]) + ", "
    f_w.write( write_byte )
    """
    if byte is not np.uint8(10): # 10 = '\n'
    else:
        print( "Reached newline." )

    if len(byteOf) == 2:
        write_byte =  "0x" + str(byteOf[0])+str(byteOf[1])+ ", "#"0x" + chr(byteOf[0]) + chr(byteOf[1]) + ", "
        #print( write_byte )
        f_w.write( write_byte )
        byteOf = [ ] # Clear array
    """
f_w.close( )

"""
for n in range(0, byte_arr.size, 2 ):
    write_byte = "0x" + chr(byte_arr[n])
    if ( n + 1 < byte_arr.size ):
        write_byte += chr(byte_arr[n+1]) + ", "
    f_w.write( write_byte )

"""
    


    

