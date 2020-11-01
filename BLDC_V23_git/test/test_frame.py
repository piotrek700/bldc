import uuid
import pycparser
from pycparser import c_parser, c_generator, c_ast, parse_file
import re
import subprocess
import cffi

def load(filename):
    name = filename + '_' + uuid.uuid4().hex
    name = 'atomic'
    source = open(filename + '.c').read()

    # preprocess all header files for CFFI
    includes = preprocess(''.join(re.findall('\s*#include\s+.*', source)))
   
    # prefix external functions with extern "Python+C"
    local_functions = FunctionList(preprocess(source)).funcs

    includes = convert_function_declarations(includes, local_functions)

    ffibuilder = cffi.FFI()
    #print(includes)

    includes = """#pragma pack(push,_CRT_PACKING)
    typedef char *__gnuc_va_list;
    typedef __gnuc_va_list va_list;
    #pragma pack(pop)
    void __debugbreak(void);
   

    extern "Python+C" const char *__mingw_get_crt_info(void);
    #pragma pack(push,_CRT_PACKING)
    typedef unsigned long long size_t;
    typedef long long ssize_t;
    typedef size_t rsize_t;
    typedef long long intptr_t;
    typedef unsigned long long uintptr_t;
    typedef long long ptrdiff_t;
    typedef unsigned short wchar_t;
    typedef unsigned short wint_t;
    typedef unsigned short wctype_t;
    typedef int errno_t;
    typedef long __time32_t;
    typedef long long __time64_t;
    typedef __time64_t time_t;
    struct threadlocaleinfostruct;
    struct threadmbcinfostruct;
    typedef struct threadlocaleinfostruct *pthreadlocinfo;
    typedef struct threadmbcinfostruct *pthreadmbcinfo;
    struct __lc_time_data;


    #pragma pack(pop)
    extern "Python+C" extern int *_errno(void);
    extern "Python+C" errno_t _set_errno(int _Value);
    extern "Python+C" errno_t _get_errno(int *_Value);
    extern "Python+C" extern unsigned long __threadid(void);
    extern "Python+C" extern uintptr_t __threadhandle(void);
    typedef struct
    {
      long long __max_align_ll;
      long double __max_align_ld;
    } max_align_t;
    typedef signed char int8_t;
    typedef unsigned char uint8_t;
    typedef short int16_t;
    typedef unsigned short uint16_t;
    typedef int int32_t;
    typedef unsigned uint32_t;
    typedef long long int64_t;
    typedef unsigned long long uint64_t;
    typedef signed char int_least8_t;
    typedef unsigned char uint_least8_t;
    typedef short int_least16_t;
    typedef unsigned short uint_least16_t;
    typedef int int_least32_t;
    typedef unsigned uint_least32_t;
    typedef long long int_least64_t;
    typedef unsigned long long uint_least64_t;
    typedef signed char int_fast8_t;
    typedef unsigned char uint_fast8_t;
    typedef short int_fast16_t;
    typedef unsigned short uint_fast16_t;
    typedef int int_fast32_t;
    typedef unsigned int uint_fast32_t;
    typedef long long int_fast64_t;
    typedef unsigned long long uint_fast64_t;
    typedef long long intmax_t;
    typedef unsigned long long uintmax_t;
    void enter_critical(void);
    void exit_critical(void);
    uint32_t critical_get_max_queue_depth(void);
    void safe_increment(uint32_t *addr);
    void safe_decrement(uint32_t *addr);
    extern "Python+C" float fast_inv_sqrtf(float x);
    extern "Python+C" float fast_atan2f(float y, float x);
    extern "Python+C" float fast_atan2f_sec(float y, float x);
    extern "Python+C" float fast_log(float val);
    extern "Python+C" float fast_norm_angle_rad(float angle);
    extern "Python+C" float fast_norm_angle_deg(float angle);

    #pragma pack(push,_CRT_PACKING)
    """

    print(includes)
 
    ffibuilder.cdef(includes)

    ffibuilder.set_source(name, source)

    ffibuilder.compile()

    module = importlib.import_module(name)
    # return both the library object and the ffi object
    return module.lib, module.ffi

class FunctionList(pycparser.c_ast.NodeVisitor):
    def __init__(self, source):
        self.funcs = set()
        self.visit(pycparser.CParser().parse(source))
        
    def visit_FuncDef(self, node):
        self.funcs.add(node.decl.name)

class CFFIGenerator(pycparser.c_generator.CGenerator):
    def __init__(self, blacklist):
        super().__init__()
        self.blacklist = blacklist
        
    def visit_Decl(self, n, *args, **kwargs):
        result = super().visit_Decl(n, *args, **kwargs)
        if isinstance(n.type, pycparser.c_ast.FuncDecl):
            if n.name not in self.blacklist:
                return 'extern "Python+C" ' + result
        return result

def convert_function_declarations(source, blacklist):
    return CFFIGenerator(blacklist).visit(pycparser.CParser().parse(source))


def preprocess(source):
    includes_dir = [
        #"-I../Libraries/CMSIS/Device/ST/STM32F30x/Include", 
        #"-I../Libraries/CMSIS/Include", 
        #"-I../Libraries/STM32F30x_StdPeriph_Driver/inc", 
        "-I../common/", 
        "-I../src", 
        "-I../",
        "-Ifake",
        #"-ID:/Projekty/_BLDC/02 Software/01 Embedded/CMSIS_5-5.2.0/CMSIS/DSP/Include", 
        "-DUSE_STM32F3_DISCOVERY", 
        "-DRADIO_MASTER=0", 
        "-DSILABS_RADIO_SI446X", 
        "-DARM_MATH_CM4", 
        "-DHSE_VALUE=32000000", 
        "-DSTM32F30X", 
        "-DUSE_STDPERIPH_DRIVER", 
        "-DSTM32F303xC",
        "-std=gnu11",

        '-D__attribute__(x)= ',
        #'-D__asm__(x)= ',
        '-D__asm__= ',
        '-D__extension__= ',
        #'-D__const=const ',
        '-D__inline__=inline ',
        #'-D__inline=inline ',
        '-D__volatile__(x)= ',
        #'-D__asm=asm',
        '-D__restrict__= ',
        #'-D__signed__=signed ',
        #'-D__GNUC_VA_LIST ',
        '-D__builtin_va_list=char* ',
        #'-Isomething_specific \\'

        #"-Og", 
        #"-ffunction-sections", 
        #"-fdata-sections", 
        #"-g", 
        #"-fstack-usage", 
        #"-Wall", 
        #"-Wdouble-promotion", 
        #"-Wfloat-conversion", 
        #"-ffast-math", 
        #"-funroll-loops", 
        #"-Wextra", 
        #"-Wswitch-default", 
        #"-Wswitch-enum", 
        #"-specs=nano.specs", 
    ]

    gcc_path = "E:/Programy/Qt/Tools/mingw810_64/bin/gcc"

    return subprocess.run([gcc_path, '-E', '-P', '-std=gnu11'] + includes_dir + ['-'],
                          input=source, stdout=subprocess.PIPE,
                          universal_newlines=True, check=True).stdout

#load('../src/main')
load('../common/sdk/atomic')