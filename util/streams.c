/*
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 * 
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330, Cambridge,
 * MA 02139, USA.
 * 
 */
#ident "@(#) LiS streams.c 2.23 07/30/04 10:52:59 "
#include <sys/types.h>
#undef GCOM_OPEN
#include <sys/stropts.h>
#include <sys/LiS/stats.h>
#ifdef QNX
#include <unix.h>
#define	LOOP_CLONE	"/dev/gcom/loop_clone"
#else
#define	LOOP_CLONE	"/dev/loop_clone"
#endif
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * Storage Declarations
 */

#if 0
unsigned long strstats[STRMAXSTAT][4]; /* the stats */
#else
unsigned int strstats[STRMAXSTAT][4]; /* the stats */
#endif

char	      buf[100000] ;		/* large buffer */

char		*prog_name ;

int		cflag = 0;
int		Cflag = 0;
int		mflag = 0;
int		Mflag = 0;
int		dflag = 0;
int		Dflag = 0;
int		Lflag = 0;
int		sflag = 0;
int		Sflag = 0;
int		tflag = 0;
int		Tflag = 0;
int		pflag = 0;
int		qflag = 0;
int		Hflag = 0;
int		wflag = 0;
int		Qflag = 0;
int		Pflag = 0;

long		memK = 0 ;
long		msgmemK = 0 ;

lis_qrun_stats_t	qrun_stats ;

typedef struct
{
    int		micro_secs ;
    unsigned	counter ;

} histogram_bucket_t ;

/*
 * Names of stat slots
 */
itemname_t lis_itemnames[] =
{
    {MODUSE,		MODUSESTR},
    {MEMALLOCS,		MEMALLOCSSTR},
    {MEMFREES,		MEMFREESSSTR},
    {MEMALLOCD,		MEMALLOCDSTR},
    {MEMLIM,		MEMLIMSTR},
    {HEADERS,		HEADERSSTR},
    {FREEHDRS,		FREEHDRSSTR},
    {DATABS,		DATABSSTR},
    {DBLKMEM,		DBLKMEMSTR},
    {MSGMEMLIM,		MSGMEMLIMSTR},
    {BUFCALLS,		BUFCALLSSTR},
    {MSGSQD,		MSGSQDSTR},
    {MSGQDSTRHD,	MSGQDSTRHDSTR},
    {CANPUTS,		CANPUTSSTR},
    {QUEUES,		QUEUESSTR},
    {QSCHEDS,		QSCHEDSSTR},
    {WRITECNT,		WRITESTR},
    {READCNT,		READSTR},
    {OPENTIME,		OPENTIMESTR},
    {CLOSETIME,		CLOSETIMESTR},
    {READTIME,		READTIMESTR},
    {WRITETIME,		WRITETIMESTR},
    {IOCTLTIME,		IOCTLTIMESTR},
    {GETMSGTIME,	GETMSGTIMESTR},
    {PUTMSGTIME,	PUTMSGTIMESTR},
    {POLLTIME,		POLLTIMESTR},
    {LOCKCNTS,		LOCKCNTSSTR}
};

char *lis_countnames[] =
{
    CURRENTSTR, TOTALSTR, MAXIMUMSTR, FAILURESSTR
};

char *sched_policy_strings[] = {"Round Robin Processors", 
				 "Last Processor First"};

void
LisShowStrStats(void)
{
    int i;
    int inx;
    int found_time = 0 ;
    int have_hdr = 0 ;

    if (!sflag && !tflag) return ;	/* no stats desired */

    /* Print statistics */
    for (i = 0; i < STRMAXSTAT; i++)
    {
	int j;

	if (lis_itemnames[i].name == NULL) continue ;

	inx = lis_itemnames[i].stats_inx ;
	if (inx == OPENTIME)
	{				/* time for a new header */
	    found_time = 1 ;
	    have_hdr = 0 ;
	}

	if (found_time)			/* got to time stats */
	{
	    if (!tflag) break ;		/* not supposed to print them */
	}
	else				/* still in regular stats */
	if (!sflag)			/* not supposed to print them */
	    continue ;

	if (!have_hdr)
	{
	    int	k ;
	    have_hdr = 1 ;
	    printf("\n%-28s  ", "");
	    for (k = 0; k < 4; k++)
            {
		printf("%12s", lis_countnames[k]) ;
            }

            printf("\n");
	}

	printf("%-28s: ", lis_itemnames[i].name);
	for (j = 0; j < 4; j++)
        {
            printf("%12u", strstats[inx][j]);
        }
            
        printf("\n");
    }


} /*LisShowStrStats*/

void print_qrun_stats(void)
{
int cpu;
unsigned int cpu_mask;
unsigned long setqsched_total = 0;
unsigned long setqsched_isr_total = 0;
unsigned long setqsched_usr_total = 0;
unsigned long setqsched_defer_total = 0;
unsigned long runq_cnts_total = 0;
unsigned long runq_srv_rpt_cnts_total = 0;
unsigned long runq_pokes_total = 0;
unsigned long runq_wakeups_total = 0;
unsigned long queuerun_total = 0;
double ave;

	printf("Sched Policy: %s\n",
		sched_policy_strings[qrun_stats.sched_policy]) ;
	cpu_mask = 0;
	for (cpu = 0; cpu < qrun_stats.num_cpus; cpu++)
		{
		if (qrun_stats.runq_pids[cpu] != 0)
			cpu_mask |= 1 << cpu;
		}
	
	printf("N-CPUs   CPU-MASK N-Qrunners N-Running N-Requested RunQ-Thresh\n"
	"%6lu %#10x %10lu %9lu %11lu %11lu\n",
		qrun_stats.num_cpus, cpu_mask, qrun_stats.num_qrunners,
		qrun_stats.queues_running, qrun_stats.runq_req_cnt,
		(qrun_stats.work_threshold ? qrun_stats.work_threshold : 16));

	printf(
"\n"
"Legend:\n"
"Qsched-Cnts: Total number of times lis_setqsched was called.\n"
"Qsched-ISR:  Number of times lis_setqsched was called from interrupt context.\n"
"Qsched-Defr: Number of times lis_setqsched was called from qenable().\n"
"Qrun-Usr:    Number of times queuerun() was called from a user space thread\n"
"Svc-Q-Cnts:  Number of service routines run.\n"
"Svc-Q-Rpts:  Number of times a service routine was re-run in queuerun().\n"
"Qrun-Pokes:  Number of times queue runner thread was poked to wake up.\n"
"Qrun-Wkups:  Number of times queue runner thread actually woke up.\n"
"Qrun-Cnts:   Number of times queuerun() was called.\n"
"Active:      1 if currently in queuerun().\n"
"Thread-PID:  PID of queue runner thread bound to CPU.\n"
"Ave Svc/QR:  Average number of service routines run per queuerun().\n");
    printf(
"\n"
"CPU   Qsched-Cnts Qsched-ISR Qsched-Defr Svc-Q-Cnts Svc-Q-Rpts  Qrun-Usr Qrun-Pokes Qrun-Wkups Qrun-Cnts Active Thread-PID"
"\n"
          ) ;
    for (cpu = 0; cpu < qrun_stats.num_cpus; cpu++)
    {
	printf("%3u   %11lu %10lu %11lu %10lu %9lu %10lu %10lu %9lu %9lu %6lu %10lu\n",
		cpu,
		qrun_stats.setqsched_cnts[cpu],
		qrun_stats.setqsched_isr_cnts[cpu],
		qrun_stats.setqsched_defer_cnts[cpu],
		qrun_stats.runq_cnts[cpu],
		qrun_stats.runq_srv_rpt_cnts[cpu],
		qrun_stats.setqsched_usr_cnts[cpu],
		qrun_stats.setqsched_poke_cnts[cpu],
		qrun_stats.runq_wakeups[cpu],
		qrun_stats.queuerun_cnts[cpu],
		qrun_stats.active_flags[cpu], 
		qrun_stats.runq_pids[cpu]) ;

		setqsched_total += qrun_stats.setqsched_cnts[cpu];
		setqsched_isr_total += qrun_stats.setqsched_isr_cnts[cpu];
		setqsched_defer_total += qrun_stats.setqsched_defer_cnts[cpu];
		setqsched_usr_total += qrun_stats.setqsched_usr_cnts[cpu];
		runq_cnts_total += qrun_stats.runq_cnts[cpu],
		runq_srv_rpt_cnts_total += qrun_stats.runq_srv_rpt_cnts[cpu],
		runq_pokes_total += qrun_stats.setqsched_poke_cnts[cpu];
		runq_wakeups_total += qrun_stats.runq_wakeups[cpu];
		queuerun_total += qrun_stats.queuerun_cnts[cpu];
    }
    printf(
"\n"
"      Qsched-Cnts Qsched-ISR Qsched-Defr Svc-Q-Cnts Svc-Q-Rpts  Qrun-Usr Qrun-Pokes Qrun-Wkups Qrun-Cnts \n"
          ) ;
    printf("Total %11lu %10lu %11lu %10lu %9lu %10lu %10lu %9lu %9lu\n",
	setqsched_total, setqsched_isr_total, 
	setqsched_defer_total, runq_cnts_total, runq_srv_rpt_cnts_total,
	setqsched_usr_total, runq_pokes_total, runq_wakeups_total, 
	queuerun_total);

    printf(
"\n"
"CPU   Ave Svc/QR \n"
"\n");
    for (cpu = 0; cpu < qrun_stats.num_cpus; cpu++)
    {
	if (qrun_stats.queuerun_cnts[cpu])
		ave = ((double)qrun_stats.runq_cnts[cpu] /
		(double)qrun_stats.queuerun_cnts[cpu]);
	else
		ave = 0.0;

	printf("%3u   %10.2f\n",
		cpu, ave);
    }
}

void print_sem_time_hist(char *buf, int nbytes)
{
    histogram_bucket_t	*p = (histogram_bucket_t *)buf ;

    for (; p->micro_secs != 0; p++)
    {
	if (p->counter != 0)
	    printf("%8d %12u\n", p->micro_secs, p->counter) ;
    }

    if (p->counter != 0)
	printf("%8s %12u\n", "Larger", p->counter) ;
}

void print_debug_bits(void)
{
    printf("-d<debug-bits -- \n") ;
    printf("DEBUG_OPEN          0x00000001\n");
    printf("DEBUG_CLOSE         0x00000002\n");
    printf("DEBUG_READ          0x00000004\n");
    printf("DEBUG_WRITE         0x00000008\n");
    printf("DEBUG_IOCTL         0x00000010\n");
    printf("DEBUG_PUTNEXT       0x00000020\n");
    printf("DEBUG_STRRPUT       0x00000040\n");
    printf("DEBUG_SIG           0x00000080\n");
    printf("DEBUG_PUTMSG        0x00000100\n");
    printf("DEBUG_GETMSG        0x00000200\n");
    printf("DEBUG_POLL          0x00000400\n");
    printf("DEBUG_LINK          0x00000800\n");
    printf("DEBUG_MEAS_TIME     0x00001000\n");
    printf("DEBUG_MEM_LEAK      0x00002000\n");
    printf("DEBUG_FLUSH         0x00004000\n");
    printf("DEBUG_FATTACH       0x00008000\n");
    printf("DEBUG_SAFE          0x00010000\n");
    printf("DEBUG_TRCE_MSG      0x00020000\n");
    printf("DEBUG_CLEAN_MSG     0x00040000\n");
    printf("DEBUG_SPL_TRACE     0x00080000\n");
    printf("DEBUG_MP_ALLOC      0x00100000\n");
    printf("DEBUG_MP_FREEMSG    0x00200000\n");
    printf("DEBUG_MALLOC        0x00400000\n");
    printf("DEBUG_MONITOR_MEM   0x00800000\n");
    printf("DEBUG_DMP_QUEUE     0x01000000\n");
    printf("DEBUG_DMP_MBLK      0x02000000\n");
    printf("DEBUG_DMP_DBLK      0x04000000\n");
    printf("DEBUG_DMP_STRHD     0x08000000\n");
    printf("DEBUG_VOPEN         0x10000000\n");
    printf("DEBUG_VCLOSE        0x20000000\n");
    printf("DEBUG_ADDRS         0x80000000\n");
    printf("-D<debug-bits -- \n") ;
    printf("DEBUG_SNDFD         0x00000001\n");
    printf("DEBUG_CP (code path)0x00000002\n");
    printf("DEBUG_CACHE         0x00000004\n");
    printf("DEBUG_LOCK_CONTENTION 0x00000008\n");
    printf("DEBUG_REFCNTS       0x00000010\n");
    printf("DEBUG_SEMTIME       0x00000020\n");

} /* print_debug_bits */

void print_options(void)
{
    printf("%s <options> [start] [stop]\n", prog_name) ;
    printf("  -c[<kb>] Print/set max message memory usage for LiS\n");
    printf("  -C[<kb>] Print/set max memory usage for LiS\n");
    printf("  -d<msk>  Debug mask (in hex)\n");
    printf("  -D<msk>  Second debug mask (in hex)\n");
    printf("  -s       Print streams memory stats\n");
    printf("  -S       Print streams queue run stats\n");
    printf("  -L       Print lock contention table\n");
    printf("  -m       Print memory allocation to log file (from kernel)\n");
    printf("  -M       Print buffer related memory allocation to log file (from kernel)\n"
	   "           0 = Display ALL\n"
	   "           n = Number to display\n");
    printf("  -p       Print SPL trace buffer to log file (from kernel)\n");
    printf("  -P       Change queue runner thread scheduling policy\n"
	   "           0 = Round Robin Processor.  Start queue runner\n"
	   "               threads starting from processor where we left off \n"
	   "               last time.\n"
	   "           1 = Last Processor First.  Always start from the \n"
	   "               highest numbered processor and work backwards \n"
	   "               until we have enough queue runner threads.\n");
    printf("  -q       Print all queues to log file (from kernel)\n");
    printf("  -t       Print streams timing stats\n");
    printf("  -T       Print semaphore timing histogram\n");
    printf("  -Q       Set the CPUS on which Queue runner threads can be run.\n"
           "           CPUs are specified as a hexadecimal bitmask where\n"
           "           the bit position specifies the CPU number. IE -Q 0x4 to \n"
           "           only allow CPU 2 to run its queue runner thread.  \n"
           "           (default=all running CPUS)\n");
    printf("  -w       Set the maximum number of service routines which should be \n"
           "           queued before starting a new queue runner (default=16).\n");
    printf("  start    Starts the streams subsystem\n");
    printf("  stop     Stops the streams subsystem\n");
    printf("  status   Reports status of streams subsystem\n");

    if (Hflag)
	print_debug_bits() ;

} /* print_options */

int main( int argc, char *argv[])
{
    int		fd ;
    int		rslt ;
    int		c;
    unsigned long debug_mask;
    unsigned long debug_mask2;
    extern char *optarg;
    unsigned int work_incr;
    unsigned int cpus_allowed;
    unsigned int policy_type;
    unsigned int num_bufs = 0;

    prog_name = argv[0] ;

    /* 
     * Check for mnemonic options.  These must appear alone
     * after the command name.
     */
    if (argc >= 2 && argv[1] != NULL)
    {
	if (strcmp(argv[1], "start") == 0)
	{
	    exit( system("/usr/sbin/strms_up") );
	}

	if (strcmp(argv[1], "stop") == 0)
	{
	    exit( system("/usr/sbin/strms_down") );
	}

	if (strcmp(argv[1], "status") == 0)
	{
	    exit( system("/usr/sbin/strms_status") );
	}
    }
    
    while(( c = getopt(argc, argv, "LpqsStTmhHw:Q:P:M:d:D:c::C::")) != -1)
    {
	switch (c)
	{
	    case 'c':
	        cflag = 1;
		if (optarg != NULL)
		{
		    if ( sscanf(optarg, "%li", &msgmemK) != 1 )
		    {
			printf("invalid argument for -c: \"%s\"\n", optarg);
			exit(1);
		    }
		}
		break ;

	    case 'C':
	        Cflag = 1;
		if (optarg != NULL)
		{
		    if ( sscanf(optarg, "%li", &memK) != 1 )
		    {
			printf("invalid argument for -C: \"%s\"\n", optarg);
			exit(1);
		    }
		}
		break ;

	    case 'd':
	        dflag = 1;
	        if ( sscanf(optarg, "%lx", &debug_mask) != 1 )
	        {
	            printf("need hex argument for -d\n");
	            exit(1);
	        }
		break;

	    case 'D':
	        Dflag = 1;
	        if ( sscanf(optarg, "%lx", &debug_mask2) != 1 )
	        {
	            printf("need hex argument for -D\n");
	            exit(1);
	        }
		break;

	    case 'L':
		Lflag = 1;
		break ;

	    case 's':
	        sflag = 1;
	        break;

	    case 'S':
	        Sflag = 1;
	        break;

	    case 't':
	        tflag = 1;
	        break;

	    case 'T':
	        Tflag = 1;
	        break;

	    case 'm':
	        mflag = 1;
	        break;

            case 'M':
	        Mflag = 1;
		num_bufs = strtol(optarg, NULL, 0);
		break ;

	    case 'p':
	    	pflag = 1 ;
		break ;

	    case 'q':
	    	qflag = 1 ;
		break ;

	    case 'w':
	        wflag = 1;
		if ( sscanf(optarg, "%u", &work_incr) != 1 )
			{
			printf("invalid argument for -w: \"%s\"\n", optarg);
			exit(1);
			}
		break ;

	    case 'Q':
	        Qflag = 1;
		cpus_allowed = strtol(optarg, NULL, 0);
		break ;

	    case 'P':
	        Pflag = 1;
		policy_type = strtol(optarg, NULL, 0);
		break ;

	    case 'H':
		Hflag = 1 ;
	    case 'h':
	    case '?':
		print_options() ;
		exit(1);
	    default:
		printf("Invalid argument %c\n", c);
		print_options() ;
		exit(1);
	}
    }

    if (!mflag && !Mflag && !dflag && !sflag && !tflag && !pflag && !qflag
	&& !cflag && !Cflag && !Sflag && !Dflag && !Lflag && !Tflag
	&& !wflag && !Qflag && !Pflag)
    {
	print_options() ;
	exit(1);
    }

    /******************************************************************/
    /* Check if LiS is running and don't issue commands if it is not. */
    /* This is to avoid LiS starting, which can cause a panic if it   */
    /* is started while LiS is going down (stopped).                  */
    /******************************************************************/

    if((system("lsmod 2>/dev/null | grep -q -w streams")))
    {
            printf("\nLiS is not running.\n\n");
            return 0;
    }
    
    fd = open(LOOP_CLONE, 0, 0) ;
    if (fd < 0)
    {
	printf(LOOP_CLONE ": %s\n", strerror(errno)) ;
	exit(1) ;
    }

    if ( dflag )
    {
	rslt = ioctl(fd, I_LIS_SDBGMSK, debug_mask) ;
	if (rslt < 0)
	{
	    printf(LOOP_CLONE ": I_LIS_SDBGMSK: %s\n", strerror(errno)) ;
	    exit(1) ;
	}
    }

    if ( Dflag )
    {
	rslt = ioctl(fd, I_LIS_SDBGMSK2, debug_mask2) ;
	if (rslt < 0)
	{
	    printf(LOOP_CLONE ": I_LIS_SDBGMSK2: %s\n", strerror(errno)) ;
	    exit(1) ;
	}
    }

    if ( sflag || tflag )
    {
	/* printf("sizeof strstats: %d\n", sizeof(strstats)); */
	rslt = ioctl(fd, I_LIS_GETSTATS, strstats) ;
	if (rslt < 0)
	{
	    printf(LOOP_CLONE ": I_LIS_GETSTATS: %s\n", strerror(errno)) ;
	    exit(1) ;
	}
	LisShowStrStats();
    }

    if ( Sflag )
    {
	rslt = ioctl(fd, I_LIS_QRUN_STATS, &qrun_stats) ;
	if (rslt < 0)
	{
	    printf(LOOP_CLONE ": I_LIS_QRUN_STATS: %s\n", strerror(errno)) ;
	    exit(1) ;
	}
	print_qrun_stats() ;
    }

    if ( mflag )
    {
	rslt = ioctl(fd, I_LIS_PRNTMEM, 0) ;
	if (rslt < 0)
	{
	    printf(LOOP_CLONE ": I_LIS_PRNTMEM: %s\n", strerror(errno)) ;
	    exit(1) ;
	}
#ifdef QNX
	printf("The memory dump is in the /usr/lib/gcom/streams.log file\n");
#endif
    }

    if ( Mflag )
    {
	rslt = ioctl(fd, I_LIS_PRNTBUF, num_bufs) ;
	if (rslt < 0)
	{
	    printf("I_LIS_PRNTBUF ioctl failed errno %d %s\n", 
		   errno, strerror(errno)) ;
	    exit(1) ;
	}
    }
    
    if ( pflag )
    {
	rslt = ioctl(fd, I_LIS_PRNTSPL, 0) ;
	if (rslt < 0)
	{
	    printf(LOOP_CLONE ": I_LIS_PRNTSPL: %s\n", strerror(errno)) ;
	    exit(1) ;
	}
    }

    if ( qflag )
    {
	rslt = ioctl(fd, I_LIS_PRNTQUEUES, 0) ;
	if (rslt < 0)
	{
	    printf(LOOP_CLONE ": I_LIS_PRNTQUEUES: %s\n", strerror(errno)) ;
	    exit(1) ;
	}
    }

    if ( cflag )
    {
	if (msgmemK)
	{
	    rslt = ioctl(fd, I_LIS_SET_MAXMSGMEM, msgmemK * 1024) ;
	    if (rslt < 0)
	    {
		perror("ioctl I_LIS_SET_MAXMSGMEM") ;
		exit(1) ;
	    }

	}

	rslt = ioctl(fd, I_LIS_GET_MAXMSGMEM, &msgmemK) ;
	if (rslt < 0)
	{
	    perror("ioctl I_LIS_GET_MAXMSGMEM") ;
	    exit(1) ;
	}

	if (msgmemK == 0)
	    printf("Maximum memory to use for messages: No limit\n") ;
	else
	{
	    msgmemK /= 1024 ;
	    printf("Maximum memory to use for messages: %ldK\n", msgmemK) ;
	}
    }

    if ( Cflag )
    {
	if (memK)
	{
	    rslt = ioctl(fd, I_LIS_SET_MAXMEM, memK * 1024) ;
	    if (rslt < 0)
	    {
		perror("ioctl I_LIS_SET_MAXMEM") ;
		exit(1) ;
	    }

	}

	rslt = ioctl(fd, I_LIS_GET_MAXMEM, &memK) ;
	if (rslt < 0)
	{
	    perror("ioctl I_LIS_GET_MAXMEM") ;
	    exit(1) ;
	}

	if (memK == 0)
	    printf("Maximum total memory to use: No limit\n") ;
	else
	{
	    memK /= 1024 ;
	    printf("Maximum total memory to use: %ldK\n", memK) ;
	}
    }

    if ( Lflag )
    {
	rslt = ioctl(fd, I_LIS_LOCKS, &buf) ;
	if (rslt < 0)
	{
	    perror("ioctl I_LIS_LOCKS") ;
	    exit(1) ;
	}
	printf("%s", buf) ;
    }

    if (Tflag)
    {
	rslt = ioctl(fd, I_LIS_SEMTIME, &buf) ;
	if (rslt < 0)
	{
	    perror("ioctl I_LIS_SEMTIME") ;
	    exit(1) ;
	}
	printf("Semaphore wakeup latency histogram:\n") ;
	print_sem_time_hist(buf, rslt) ;
    }

    if ( wflag )
    {
	rslt = ioctl(fd, I_LIS_SET_WORK_INCR, &work_incr) ;
	if (rslt < 0)
	{
	    printf("I_LIS_SET_WORK_INCR ioctl failed errno %d %s\n", 
		   errno, strerror(errno)) ;
	    exit(1) ;
	}
	printf("Set work increment variable to %d\n", work_incr);
    }

    if ( Qflag )
    {
	rslt = ioctl(fd, I_LIS_SET_CPUS, &cpus_allowed) ;
	if (rslt < 0)
	{
	    printf("I_LIS_SET_CPUS ioctl failed errno %d %s\n", 
		   errno, strerror(errno)) ;
	    exit(1) ;
	}
    }

    if ( Pflag )
    {
	rslt = ioctl(fd, I_LIS_SET_POLICY, &policy_type) ;
	if (rslt < 0)
	{
	    printf("I_LIS_SET_POLICY ioctl failed errno %d %s\n", 
		   errno, strerror(errno)) ;
	    exit(1) ;
	}
    }

    close(fd) ;

    return 0;

} /* main */
