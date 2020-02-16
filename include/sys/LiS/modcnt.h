#ifndef LIS_MODCNT_H
#define LIS_MODCNT_H	1
/*
 *  This file is intended to address the different situations for which
 *  MODGET/MODPUT (i.e., increment/decrement loadable kernel module use
 *  reference counts) might be needed.
 *
 *  There are basically these kinds of C source files that might have a
 *  need:
 *  - head/linux-mdep.c - includes <linux/module.h> for the head, and
 *    thus effectively defines streams.o as a module.
 *  - head/head.c - doesn't include <linux/module.h>, but will be linked
 *    into the same streams.o object as linux-mdep.c
 *  - drivers/modules also to be linked into streams.o, and will thus not
 *    include <linux/module.h>
 *  - drivers/modules to be separate loadable modules, which will include
 *    <linux/module.h> for themselves
 *
 *  I have decided that the development versions of these routines are not
 *  easily workable.  They had to be defined in linux-mdep.c as callables,
 *  but as such, also had to be inter-module calls, whose names were
 *  exported symbols.  This is somewhat circular, considering that these
 *  routines concern the loading of modules in the first place.  Second,
 *  only the parameter-less MOD_[INC,DEC]_USE_COUNT macros were used, which
 *  makes no sense for counting references from an entirely other module.
 *  So, the development mode routines have been dropped, at least for
 *  now; they may be added back in later.  (JB 05/18/04)
 *
 *  In production mode, separate loadable modules will use the inline
 *  routines here.  Sources like head.c which are to be included in
 *  streams.o without themselves including <linux/module.h> will not be
 *  able to define such inlines, so they will call the ..._local routines,
 *  which linux-mdep.c will also define.  These need not be exported
 *  symbols, since the calls will be within streams.o.

 *  This whole thing was a bit messed up.  We cannot use the direct inline 
 *  lis_modget/put_local functions below in external loadable modules because
 *  then the module would end up directly calling a kernel symbol which is
 *  bad because of the whole regparm thing.  Therefore external modules
 *  have to call an LiS function which will call try_module_get/module_get
 *  for us.  Therefore I have invented these new functions 
 *  lis_modget/lis_modput which take a struct module *modulep parameter.
 *  This allows the module to pass _it's_ definition of THIS_MODULE so that
 *  the module gets it's reference count properly incremented.  
 *  dg@adax June 1, 2006
 */

#ident "@(#) LiS modcnt.h 1.4 09/02/04"

#if defined(_LINUX_MODULE_H)

#ifndef	_STRMDBG_H
#include <sys/LiS/strmdbg.h>	/* for debug bits */
#endif


extern void lis_modget(struct module *modulep, 
		       const char *file, int line, const char *fn) _RP;
extern void lis_modput(struct module *modulep, 
		       const char *file, int line, const char *fn) _RP;

/* External (from LiS) loadable modules should have all of these
 * defined and hense call lis_modget/lis_modput */
#if defined(LINUX) && defined(__KERNEL__) && defined(MODULE)
#define MODGET()       lis_modget(THIS_MODULE, __FILE__,__LINE__,__FUNCTION__)
#define MODPUT()       lis_modput(THIS_MODULE, __FILE__,__LINE__,__FUNCTION__)
#else
/* Drivers which are part of LiS, but which include sys/LiS/module.h get this
 * definition.  That is they are not loadable modules which #define MODULE.
 * These include: fifo loop relay clone minimux printk sad pipemod
 */
#define MODGET()
#define MODPUT()
#endif

#else

/*
 * If not a separate module then use the internal LiS modget/put via
 * the debug versions.  These reside in linux-mdep.c.
 * This is currently only called by head/head.c
 */
#define MODGET()       lis_modget_dbg(__LIS_FILE__,__LINE__,__FUNCTION__)
#define MODPUT()       lis_modput_dbg(__LIS_FILE__,__LINE__,__FUNCTION__)

#endif		/* !_LINUX_MODULE_H */



/* intra-module callables for production mode, for sources not including
 * <linux/module.h>.  these are defined in linux-mdep.c, and only wrap
 * the above ..._local inlines, as a convenience so that <linux/module.h>
 * need not be included to do module use reference counting
 */
extern void lis_modget_dbg(const char *file, int line, const char *fn) _RP;
extern void lis_modput_dbg(const char *file, int line, const char *fn) _RP;

#define LIS_MODGET()       lis_modget_dbg(__LIS_FILE__,__LINE__,__FUNCTION__)
#define LIS_MODPUT()       lis_modput_dbg(__LIS_FILE__,__LINE__,__FUNCTION__)



#endif		/* LIS_MODCNT_H */
