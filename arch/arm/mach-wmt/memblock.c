/*--- memblock.c -------------------------------------------------------------------
*   Copyright (C) 2008 WonderMedia Tech. Inc.
*
* MODULE       : memblock.c
* AUTHOR       : Jason Lin
* DATE         : 2008/12/08
* DESCRIPTION  :
*-----------------------------------------------------------------------------*/

/*--- History -------------------------------------------------------------------
*Version 0.01 , Jason Lin, 2008/12/08
* First version
*
*------------------------------------------------------------------------------*/
/* Concept:
 *          memory
 *          area 1			  memory
 *  (pfn)   (pages)           block    			pg_info
 *  +51F00+---------+ --> +------------+ +0 	=start
 *        |   ...   |     |    MB 0    | size	=(start-end)*page_size=50*4096
 *  +51F50+---------+     +------------+ +50	=end
 *        |    .    |
 *        |    .    |
 *        |    .    |
 *  +52C00+- - - - -+     +------------+ +D00	=start
 *        |   ...   |     |    MB 1    | size	=(start-end)*page_size=400*4096
 *  +53000+---------+     +------------+ +1100	=end
 *        |         |
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/mman.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/swap.h>

#include "com-mb.h"
#include <mach/memblock.h>

#define THE_MB_USER			"WMT-MB"

#define MB_UNITTEST
#ifdef MB_UNITTEST
#include "memblock-ut.h"
#endif

#define MB_DEBUG

#define MB_INFO(fmt,args...)	printk(KERN_INFO "["THE_MB_USER"] " fmt , ## args)
#define MB_WARN(fmt,args...)	printk(KERN_WARNING "["THE_MB_USER" *W*] " fmt, ## args)
#define MB_ERROR(fmt,args...)	printk(KERN_ERR "["THE_MB_USER" *E*] " fmt , ## args)

#ifdef MB_DEBUG
	#define MB_IOCTLDBG(fmt, args...)	\
		if(MBMSG_LEVEL){	\
			printk("\t"KERN_DEBUG "["THE_MB_USER"] %s: " fmt, __FUNCTION__ , ## args);	\
		}
	#define MB_DBG(fmt, args...)	\
		if(MBMSG_LEVEL > 1){	\
			printk("\t"KERN_DEBUG "["THE_MB_USER"] %s: " fmt, __FUNCTION__ , ## args);	\
		}
	#define MB_WDBG(fmt, args...)	\
		if(MBMSG_LEVEL > 2){	\
			printk("\t"KERN_DEBUG "["THE_MB_USER"] %s: " fmt, __FUNCTION__ , ## args);	\
		}
#else
	#define MB_IOCTLDBG(fmt, args...)
	#define MB_DBG(fmt, args...)
	#define MB_WDBG(fmt, args...)
#endif

#define M(x) ((x)<<20)

//#ifndef CONFIG_VIDEO_BUFFER_SIZE
//#define CONFIG_VIDEO_BUFFER_SIZE 32 * 1024
#define CONFIG_VIDEO_BUFFER_SIZE (((num_physpages << PAGE_SHIFT)>M(128))?(32*1024):(16*1024))
//#endif

#define MBA_MAX_ORDER		(MAX_ORDER - 1)
#define MBA_MIN_ORDER		(MBA_MAX_ORDER - 4)	// 2^8 pages == 1 Mb

#define MBFLAG_KERNEL		0x80000000	// kernel space allocate
#define MBFLAG_USER			0x40000000	// user space allocate

#define MBAFLAG_STATIC		0x80000000	// static, without release
										// if there is no mb inside

#define MBUSRCH_ALL			0x00000000	// search all mbu
#define MBUSRCH_CREATOR		0x00000001	// search creator
#define MBUSRCH_MMAP		0x00000002	// search mbu which mapping from user
										// of course, skip owner
#define MBUSRCH_GETPUT		0x00000003	// mbu came from get/put while mbusearch,
										// of course, skip owner

#define MB_IN_USE(mb)		(mb->count.counter || mb->creator || !list_empty(&mb->mbu_list))
#define PAGE_KB(a)			((a)*(PAGE_SIZE/1024))
#define MB_COMBIND_MBA(h,t)	\
	(h)->pgi.pfn_end = (t)->pgi.pfn_end;	\
	(h)->pages += (t)->pages;	\
	(h)->tot_free_pages += (t)->tot_free_pages;	\
	(h)->max_available_pages += (t)->max_available_pages;	\
	list_del(&((t)->mba_list));	\
	wmt_mbah->nr_mba--;	\
	kmem_cache_free(wmt_mbah->mba_cachep, t);

struct mb_user;

struct page_info{
	unsigned long			pfn_start;	// start pfn of this memory block
	unsigned long			pfn_end;	// end pfn of this memory block
};

struct mba_host_struct{
	/* MB area link list link all MBAs */
	struct list_head		mba_list;

	/* page information of this MBA */
	unsigned int			nr_mba;

	/* total pages of all manager MBAs */
	unsigned long 			tot_pages;

	/* total static pages of all manager MBAs */
	unsigned long			tot_static_pages;

	/* total free pages of all manager MBAs */
	unsigned long			tot_free_pages;

	/* max free pages of all manager MBAs */
	unsigned long	 		max_available_pages;

	/* allocator of MBA,
	   use slab to prevent memory from fragment. */
	struct kmem_cache			*mba_cachep;

	/* allocator of MB,
	   use slab to prevent memory from fragment. */
	struct kmem_cache			*mb_cachep;

	/* allocator of mb_user,
	   use slab to prevent memory from fragment. */
	struct kmem_cache			*mbu_cachep;

};

struct mb_area_struct{
	/* MB area link list link all MBAs */
	struct list_head		mba_list;

	/* link list link to all MBs
       belong to this MBA */
	struct list_head		mb_list;

	/* pointer point to dedicated MBAH */
	struct mba_host_struct	*mbah;

	/* flags of MBA */
	unsigned int			flags;

	/* prefech record task */
	pid_t					tgid;	// task pid

	/* start physical address of this MBA */
	unsigned long			start;

	/* size of this MB area. Normally,
	   MAX kernel permitted size */
	unsigned long 			pages;

	/* page information of this MBA */
	struct page_info		pgi;

	/* page information of this MBA */
	unsigned int			nr_mb;

	/* cur total free pages of this MBA */
	unsigned long 			tot_free_pages;

	/* cur max free pages of this MBA */
	unsigned long	 		max_available_pages;
};

/*
 *	element of memory block,
 *	minimal size limitation is PAGE_SIZE
 */
struct mb_struct{
	/* MB link list link all MBs
       in dedicated MBA */
	struct list_head		mb_list;

	/* pointer point to dedicated MBA */
	struct mb_area_struct	*mba;

	/* MB kernel page information */
	struct page_info		pgi;

	/* kernel physical address
	   for HW access */
	unsigned long			physical;

	/* allocate size */
	unsigned long			size;

	/* current MB use count,
       release until zero */
	atomic_t				count;

	/* special flag to control */
	unsigned int			flag;

	/* point to owner created the mb.
	   this enlisted in mbu_list */
	struct mb_user			*creator;

	/* use for trace the user of mb */
	struct list_head		mbu_list;
};

struct mb_user{
	/* user link list link all users
	   (include creator) of belonged MB */
	struct list_head		mbu_list;

	/* the mb to which this user belong */
	struct mb_struct		*mb;

	/* type 0 kernel space user
			1 user space user */
	unsigned int			type:1;

	/* owner 0 put/get the mb
			 1 owner who allocate the mb */
	unsigned int			owner:1;

	/* task to which user belonged,
	   user space user only */
	pid_t					tgid;	// task pid

	/* kernel space user: virtual address
	   user space user: mmapped - user address
	   					unmapped - physical address
	   							   because it need to be found */
	unsigned long			addr;

	/* kernel space user: mb size
	   user space user:   mmap size
	   zero: 			  owner - user space allocate but not mapped yet
	   					  not owner -come from get/put */
	unsigned long			size;

	/* user name for recoder */
	char	 				the_user[TASK_COMM_LEN];
};

static struct mba_host_struct *wmt_mbah = NULL;
struct file_operations mb_fops;
static unsigned char MBMSG_LEVEL = 0;
static unsigned char MBMAX_ORDER = 0;
static unsigned char MBMIN_ORDER = 0;

/* read/write spinlock for multientry protection. */
static spinlock_t mb_do_lock;
static spinlock_t mb_search_lock;
static spinlock_t mb_ioctl_lock;
static spinlock_t mb_task_mm_lock;

// return address is guaranteeed only under page alignment
void* user_to_virt(unsigned long user)
{
	struct vm_area_struct *vma;
	struct mm_struct *mm = current->mm;
	unsigned long from,end,pmd_end,pte_end,offset,flags;
	pgd_t * pgd;
	pmd_t * pmd;
	pte_t * pte;

	spin_lock_irqsave(&mb_task_mm_lock, flags);

	vma = find_vma(mm, user);
	if (!vma)
		goto fault;

	from = vma->vm_start;
	end = PAGE_ALIGN(vma->vm_end);
	offset = user - from;
	pgd = pgd_offset(mm, from);

	do {	// PGD
		if(!(pmd = pmd_offset(pgd, from)))
			goto fault;
		// from start to PGDIR alignment
		pmd_end = (from + PGDIR_SIZE) & PGDIR_MASK;
		pmd_end = min(pmd_end,end);
		do {	// PMD
			if(!(pte = pte_offset_map(pmd, from)))
				goto fault;
			// from start to PMD alignment
			pte_end = (from + PMD_SIZE) & PMD_MASK;
			pte_end = min(pte_end,pmd_end);
			do {	// PTE
				if(offset < PAGE_SIZE){
				    spin_unlock_irqrestore(&mb_task_mm_lock, flags);
					return page_address(pfn_to_page(pte_pfn(*pte)))+offset;
				}
				from += PAGE_SIZE;
				offset -= PAGE_SIZE;
				pte++;
			} while (from && (from < pte_end));
			pmd++;
		} while (from && (from < pmd_end));
		pgd++;
	} while (from && (from < end));

fault:
    spin_unlock_irqrestore(&mb_task_mm_lock, flags);
	return NULL;
}

int user_to_prdt(
	unsigned long user,
	unsigned int size,
	struct prdt_struct *next,
	unsigned int items)
{
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	struct prdt_struct *prev = NULL;
	unsigned long from,end,pmd_end,pte_end,offset,flags;
	unsigned int pgdidx,pmdidx,pteidx,prdidx;
	pgd_t * pgd;
	pmd_t * pmd;
	pte_t * pte;

	if(!next){
		MB_WARN("Null PRD table space.\n");
		return -EINVAL;
	}

	if(((size/PAGE_SIZE)+2) > items){
		MB_WARN("The size of PRD table is not safe. (at least %lu)\n",(size/PAGE_SIZE)+2);
		return -EINVAL;
	}

	MB_DBG("Memory(0x%lx,%d) PRDT(%p,%d)\n",user,size,next,items);

	spin_lock_irqsave(&mb_task_mm_lock, flags);

	__asm__ __volatile__ (
            "1:      mrc p15, 0, r15, c7, c14, 3 \n\t"
            "         bne 1b"
            );

	while(size > 0){
		if (!(vma = find_vma(mm, user))){
			MB_WARN("user addr %lx not found in task %s\n",user,current->comm);
			next->EDT = 1;
			goto fault;
		}

		MB_WDBG("VMA found: start %lx end %lx\n",vma->vm_start,vma->vm_end);

		from = vma->vm_start;
		end = PAGE_ALIGN(vma->vm_end);
		offset = user - from;
		pgd = pgd_offset(mm, from);
		pgdidx = prdidx = 0;
		do {	// PGD
			if(!(pmd = pmd_offset(pgd, from)))
				goto fault;
			// from start to PGDIR alignment
			pmd_end = (from + PGDIR_SIZE) & PGDIR_MASK;
			pmd_end = min(pmd_end,end);
			MB_WDBG("\t[PGD%3d] *(%p)=%lx start %lx to %lx\n",
				pgdidx++,pgd,pgd_val(*pgd),from,pmd_end);
			pmdidx = 0;
			do {	// PMD
				if(!(pte = pte_offset_map(pmd, from)))
					goto fault;
				// from start to PMD alignment
				pte_end = (from + PMD_SIZE) & PMD_MASK;
				pte_end = min(pte_end,pmd_end);
				MB_WDBG("\t\t[PMD%3d] *(%p)=%lx start %lx to %lx\n",
					pmdidx++,pmd,pmd_val(*pmd),from,pte_end);
				pteidx = 0;
				do {	// PTE
					unsigned long pfn = pte_pfn(*pte);
					struct page *pg = pfn_to_page(pfn);
					void *virt = page_address(pg);
					dma_addr_t phys = virt_to_phys(virt);

					MB_WDBG("\t\t\t[PTE%3d] PTE *(%p)=%lx pfn %lx page %p vir %p phy %x vma %lx\n",
						pteidx++,pte,pte_val(*pte),pfn,pg,virt,phys,from);
					if(offset < PAGE_SIZE){
						if(!phys){
							MB_WARN("user space %lx doesn't map to virtual space yet.\n",from);
							goto fault;
						}

						// Check it could combind with previous one
						if( prev &&
							(prev->size <= ((1 << 16) - (2 * PAGE_SIZE))) && // prd size boundary check, MAX 60K
							((prev->addr + prev->size) == phys)){ // page continuity check
							prev->size += min((unsigned long)size,PAGE_SIZE-offset);
						}
						else{	// create new one
							prev = next++;
							prev->addr = phys + offset;
							prev->size = min((unsigned long)size,PAGE_SIZE-offset);
							prdidx++;
							items--;
							if(!items){
								prev->EDT = 1;
								goto fault;
							}
						}
						size -= min((unsigned long)size,PAGE_SIZE-offset);;
						prev->reserve = 0;
						prev->EDT = (size)?0:1;
						MB_WDBG("\t\t\t\t[PRD %3d] %p start %x size %x edt %x rest %x\n",
							prdidx,prev,prev->addr,prev->size,prev->EDT,size);
						if(prev->EDT){
                            spin_unlock_irqrestore(&mb_task_mm_lock, flags);
							return 0;
						}
						offset = 0;
					}
					else
						offset -= PAGE_SIZE;

					from += PAGE_SIZE;
					pte++;
#ifdef MB_WORDY
					msleep(30);
#endif
				} while (from && (from < pte_end));
				pmd++;
			} while (from && (from < pmd_end));
			pgd++;
		} while (from && (from < end));
		user = from;
	}
fault:
    spin_unlock_irqrestore(&mb_task_mm_lock, flags);
	MB_WARN("USER TO PRDT unfinished, remain size %d\n",size);
	return -EFAULT;
}

// return physical address
static unsigned long mb_alloc_pages(unsigned long *nr_pages)
{
	unsigned long virtual = 0,physical = 0;
	unsigned int index, order;
	struct zone *zone;

	if( nr_pages == 0 ){
		MB_WARN("mb_alloc_pages zero size\n");
		return 0;
	}

	order = get_order(*nr_pages * PAGE_SIZE);
	order = max(order,(unsigned int)(MBA_MIN_ORDER));
	MB_DBG(" %ld/%d pages, order %d\n",*nr_pages, 1 << order, order);

	zone = (first_online_pgdat())->node_zones;
	for(index = order; index < MAX_ORDER; index++){
		if(zone->free_area[index].nr_free){
			virtual = __get_free_pages(GFP_KERNEL | GFP_DMA, order);
			break;
		}
	}

	if ( virtual ) {
		*nr_pages = 1 << order;
		physical = __pa(virtual);
		for(index = 0; index < *nr_pages; index++){
			SetPageReserved(virt_to_page(virtual + index * PAGE_SIZE));
		}
	}
    else {
		struct free_area *fa;
		MB_WARN("__get_free_pages fail! (pages: %d free %lu)\n",
			1 << order,(unsigned long)nr_free_pages());
		zone = (first_online_pgdat())->node_zones;
		fa = zone->free_area;
		MB_WARN("DMA ZONE: %ld*4kB %ld*8kB "
			 "%ld*16kB %ld*32kB %ld*64kB "
			 "%ld*128kB %ld*256kB %ld*512kB "
			 "%ld*1024kB %ld*2048kB %ld*4096kB "
			 "%ld*8192kB = %ldkB\n",
			 fa[0].nr_free,fa[1].nr_free,
			 fa[2].nr_free,fa[3].nr_free,fa[4].nr_free,
			 fa[5].nr_free,fa[6].nr_free,fa[7].nr_free,
			 fa[8].nr_free,fa[9].nr_free,fa[10].nr_free,
			 fa[11].nr_free,PAGE_KB(nr_free_pages()));
    }

	return physical;
}

static void mb_free_pages(
	unsigned long physical,
	unsigned int nr_pages
)
{
	unsigned long virtual;
	unsigned int index;

	if(!physical || !nr_pages){
		MB_WARN("mb_free_pages unknow addr %lx size %x\n",physical,nr_pages);
		return;
	}

	virtual = (unsigned long)__va(physical);
	for(index = 0; index < nr_pages; index++){
		ClearPageReserved(virt_to_page(virtual + index * PAGE_SIZE));
	}

	free_pages(virtual, get_order(nr_pages * PAGE_SIZE));
}

static int mb_show_mb(
	struct mb_struct *mb,
	char *msg,
	char *str,
	int len
)
{
	struct mb_user *mbu;
	char buffer[1024],*p;

	if(!str && MBMSG_LEVEL > 1)
		return 0;

	if(!mb)
		return 0;

	memset(buffer,0x0,1024);
	p = buffer;
	p += sprintf(p,"%s%s[MB,%p] %08lx %4ldkb [%4lx,%4lx] %3x %08x\n",
					msg?msg:"",msg?" ":"",mb,mb->physical,mb->size/1024,
					mb->pgi.pfn_start,mb->pgi.pfn_end,mb->count.counter,mb->flag);

	if(!list_empty(&mb->mbu_list)){
		list_for_each_entry(mbu, &mb->mbu_list, mbu_list){
			if((1024 - (p - buffer)) < 30){
				p += sprintf(p,	"\n\n                   more ...\n");
				break;
			}
			p += sprintf(p,"                   [%p] %08lx %4ldkb %s TGID %d %s %s\n",
						mbu,mbu->addr,mbu->size/1024,mbu->type?"U":"K",mbu->tgid,
						mbu->owner?"Creator":"       ",mbu->the_user);
		}
	}

	p += sprintf(p,"\n");

	if(str){
		if(strlen(buffer) < len)
			len = strlen(buffer);
		strncpy(str, buffer, len);
		return len;
	}
	else{
		MB_DBG("%s",buffer);
	}

	return strlen(buffer);
}

static int mb_show_mba(
	struct mb_area_struct *mba,
	char *msg,
	char *str,
	int len,
	int follow
)
{
	struct mb_struct *mb;
	char buffer[4096],*p;
	int idx = 1;

	if(!str && MBMSG_LEVEL > 1)
		return 0;

	if(!mba)
		return 0;

	memset(buffer,0x0,4096);

	p = buffer;
	if(msg)
		p += sprintf(p,"%s ",msg);
	p += sprintf(p,"[MB Area,%p] %08lx %6ldkb [%4lx,%4lx] %3x %6ldkb/%6ldkb %s\n",
					mba,mba->start,PAGE_KB(mba->pages),
					mba->pgi.pfn_start,mba->pgi.pfn_end,
					mba->nr_mb,
					PAGE_KB(mba->max_available_pages),
					PAGE_KB(mba->tot_free_pages),
					(mba->flags | MBAFLAG_STATIC)?"static":"dynamic");

	if(follow){
		p += sprintf(p,	"\tindex -    [MemBlock]     phys   size [  zs,  ze] cnt     flag\n");
		list_for_each_entry(mb, &mba->mb_list, mb_list){
			if((4096 - (p - buffer)) < 18){
				p += sprintf(p,	"\n\n        more ...\n");
				break;
			}
			p += sprintf(p,"\t%5d - ",idx++);
			p += mb_show_mb(mb,NULL,p,4096 - (p - buffer));
		}
	}

	if(str){
		if(strlen(buffer) < len)
			len = strlen(buffer);
		strncpy(str, buffer, len);
		return len;
	}
	else{
		MB_DBG("%s",buffer);
	}

	return strlen(buffer);
}

static void mb_update_mbah(void)
{
	struct mba_host_struct *mbah = wmt_mbah;
	unsigned long mbah_free,mbah_max;
	unsigned long mba_free,mba_max;
	struct mb_area_struct *mba;
	struct mb_struct *mb;

	if(!mbah){
		MB_WARN("mb_update_mbah unknow mbah\n");
		return;
	}

	mbah_free = mbah_max = 0;
	list_for_each_entry(mba, &mbah->mba_list, mba_list){
		unsigned long zs,nr_pages;	// zone start

		zs = mba->pgi.pfn_start;
		mba_free = mba_max = nr_pages = 0;
		list_for_each_entry(mb, &mba->mb_list, mb_list){
			nr_pages = max(mb->pgi.pfn_start - zs,nr_pages);
			mba_free += mb->pgi.pfn_start - zs;
			zs = mb->pgi.pfn_end;
		}
		mba_free += mba->pgi.pfn_end - zs;
		mbah_free += mba_free;
		mba_max = max(mba->pgi.pfn_end - zs,nr_pages);
		mbah_max = max(mbah_max,mba_max);
	}

	mbah->tot_free_pages = mbah_free;
	mbah->max_available_pages = mbah_max;

	return;
}

static struct mb_area_struct * mb_allocate_mba(unsigned int pages)
{
	struct mba_host_struct *mbah = wmt_mbah;
	struct mb_area_struct *mba;
	unsigned long pgnum;

	if(!mbah){
		MB_WARN("mb_allocate_mba null mbah\n");
		return NULL;
	}

	pgnum = 1 << get_order(pages * PAGE_SIZE);
	if(pgnum > (1 << MBMAX_ORDER)){
		MB_WARN("mb_allocate_mba page out of size, %d/%ld/%d\n",
			pages,pgnum,(1<<MBMAX_ORDER));
		return NULL;
	}

	mba = kmem_cache_alloc(mbah->mba_cachep, GFP_KERNEL);
	if(!mba){
		MB_WARN("mb_allocate_mba out of memory\n");
		return NULL;
	}

	memset(mba,0x0,sizeof(struct mb_area_struct));
	mba->pages = pgnum;
	mba->start = mb_alloc_pages(&mba->pages);
	if(!mba->start){
		MB_WARN("mb_allocate_mba no available space\n");
		kmem_cache_free(mbah->mba_cachep, mba);
		return NULL;
	}

	// initialization
	INIT_LIST_HEAD(&mba->mb_list);
	INIT_LIST_HEAD(&mba->mba_list);
	mba->mbah = mbah;
	mba->tot_free_pages = mba->max_available_pages = mba->pages;
	mba->pgi.pfn_start = mba->start >> PAGE_SHIFT;
	mba->pgi.pfn_end = mba->pgi.pfn_start + mba->pages;
	list_add_tail(&mba->mba_list, &mbah->mba_list);

	// update MBA host
	mbah->nr_mba++;
	mbah->tot_pages += mba->pages;
	mbah->tot_free_pages += mba->tot_free_pages;
	mbah->max_available_pages =
		max(mbah->max_available_pages,mba->max_available_pages);

	mb_show_mba(mba,"allocate", NULL, 0, 0);

	MB_DBG("MBA(%p) 0x%lx ~ 0x%lx\n",mba , mba->pgi.pfn_start << PAGE_SHIFT, mba->pgi.pfn_end << PAGE_SHIFT);

	return mba;
}

static struct mb_struct * mb_allocate_mb(struct mb_area_struct *mba, unsigned long size)
{
	struct mba_host_struct *mbah;
	struct mb_struct *mb,*entry;
	struct list_head *next;
	unsigned long pages,zs,ze;

	if(!mba || !mba->mbah || !size){
		MB_WARN("mb_allocate_mb unknow arg.(%p,%p,%lx)\n",
			mba,mba?mba->mbah:NULL,size);
		return NULL;
	}

	mbah = mba->mbah;
	size = PAGE_ALIGN(size);
	pages = size >> PAGE_SHIFT;

	// mb free size is not enough
	if(mba->max_available_pages < pages){
		MB_WARN("mb_allocate_mb no available space in MBA (%lx<%lx)\n",
			pages,mba->max_available_pages);
		return NULL;
	}

	// search available zone
	// zone start from mba start
	ze = zs = mba->pgi.pfn_start;
	next = &mba->mb_list;
	list_for_each_entry(entry, &mba->mb_list, mb_list){
		next = &entry->mb_list;
		ze = entry->pgi.pfn_start;
		if((ze - zs) >= pages)
			break;
		zs = entry->pgi.pfn_end;
	}

	if(zs >= ze){
		next = &mba->mb_list;
		ze = mba->pgi.pfn_end;
	}

	// impossible, something wrong
	if((ze - zs) < pages){
		MB_WARN("something wrong in MBA %p when allocate MB.\n",mba);
		return NULL;
	}

	MB_DBG("Zone finding start %lx end %lx size %lx for size %lx\n",
		zs,ze,ze - zs,pages);

	mb = kmem_cache_alloc(mbah->mb_cachep, GFP_KERNEL);
	if(!mb){
		MB_WARN("mb_allocate_mb mba_cachep out of memory\n");
		return NULL;
	}

	memset(mb, 0x0, sizeof(struct mb_struct));
	INIT_LIST_HEAD(&mb->mb_list);
	INIT_LIST_HEAD(&mb->mbu_list);
	mb->mba = mba;
	mb->pgi.pfn_start = zs;
	mb->pgi.pfn_end = mb->pgi.pfn_start + pages;

	mb->size = size;
	mb->physical = mba->start + ((zs - mba->pgi.pfn_start) << PAGE_SHIFT);
	list_add_tail(&mb->mb_list, next);
	mba->nr_mb++;
	mba->tot_free_pages -= pages;
	mbah->tot_free_pages -= pages;

	// update mba
	zs = mba->pgi.pfn_start;
	mba->max_available_pages = 0;
	list_for_each_entry(entry, &mba->mb_list, mb_list){
		mba->max_available_pages =
			max(entry->pgi.pfn_start - zs,mba->max_available_pages);
		zs = entry->pgi.pfn_end;
	}
	mba->max_available_pages =
		max(mba->pgi.pfn_end - zs,mba->max_available_pages);

	// update mbah
	mbah->max_available_pages = 0;
	list_for_each_entry(mba, &mbah->mba_list, mba_list){
		mbah->max_available_pages =
			max(mbah->max_available_pages,mba->max_available_pages);
	}

	mb_show_mb(mb,"AllocMB",NULL,0);

	return mb;
}

static int mb_free_mba(struct mb_area_struct *mba)
{
	struct mba_host_struct *mbah;
	struct mb_area_struct *entry;

	if(!mba || !mba->mbah || mba->nr_mb){
		MB_WARN("mb_free_mba unknow arg.(%p,%p,%x)\n",
			mba,mba?mba->mbah:NULL,mba?mba->nr_mb:0);
		return -EFAULT;
	}

	mbah = mba->mbah;

	list_for_each_entry(entry, &mbah->mba_list, mba_list){
		if(entry == mba)
			break;
	}

	if(entry != mba){
		MB_WARN("mb_free_mba unknow MBA %p\n",mba);
		return -EFAULT;
	}

	if(mba->flags & MBAFLAG_STATIC)
		return 0;

	mb_show_mba(mba,"ReleaseMBA", NULL, 0, 0);

	// free mba
	list_del(&mba->mba_list);
	mb_free_pages(mba->start,mba->pages);
	mbah->nr_mba--;
	mbah->tot_free_pages -= mba->tot_free_pages;
	mbah->tot_pages -= mba->pages;
	kmem_cache_free(mbah->mba_cachep, mba);
	mba = NULL;

	// update max mb size
	mbah->max_available_pages = 0;
	list_for_each_entry(entry, &mbah->mba_list, mba_list){
		mbah->max_available_pages =
			max(mbah->max_available_pages,entry->max_available_pages);
	}

	return 0;
}

static int mb_free_mb(struct mb_struct *mb)
{
	unsigned long zs,nr_pages;
	struct mba_host_struct *mbah;
	struct mb_area_struct *mba;
	struct mb_struct *entry;

	if(!mb){
		MB_WARN("mb_free_mb unknow MB %p.\n",mb);
		return -EFAULT;
	}

	if(MB_IN_USE(mb)){
		MB_WARN("mb_free_mb invalid arg.(%p,%x,%lx,%lx,%08x)\n",
			mb,mb->count.counter,mb->physical,mb->size,mb->flag);
		return -EINVAL;
	}

	mba = mb->mba;
	if(!mba || !mba->mbah){
		MB_WARN("mb_free_mb unknow para.(%p,%p)\n",mba,mba?mba->mbah:NULL);
		return -EFAULT;
	}

	list_for_each_entry(entry, &mba->mb_list, mb_list){
		if(entry == mb)
			break;
	}

	if(entry != mb){
		MB_WARN("mb_free_mb unknow MB %p\n",mb);
		return -EFAULT;
	}

	mbah = mba->mbah;

	mb_show_mb(mb,"Retrieve unused MB",NULL,0);

	// free mb
	list_del(&mb->mb_list);
	kmem_cache_free(mbah->mb_cachep, mb);
	mba->nr_mb--;
	mb = NULL;

	// unused mba, release it
	if(!mba->nr_mb && !(mba->flags & MBAFLAG_STATIC))
		return mb_free_mba(mba);

	// update max mb size and free mb size
	mbah->tot_free_pages -= mba->tot_free_pages;	// sub old one and then add new one
	zs = mba->pgi.pfn_start;
	mba->tot_free_pages = mba->max_available_pages = nr_pages = 0;
	list_for_each_entry(entry, &mba->mb_list, mb_list){
		nr_pages = max(entry->pgi.pfn_start - zs,nr_pages);
		mba->tot_free_pages += entry->pgi.pfn_start - zs;
		zs = entry->pgi.pfn_end;
	}
	mba->tot_free_pages += mba->pgi.pfn_end - zs;
	mba->max_available_pages = max(mba->pgi.pfn_end - zs,nr_pages);
	mbah->tot_free_pages += mba->tot_free_pages;	// add new one
	mbah->max_available_pages =
		max(mbah->max_available_pages,mba->max_available_pages);

	return 0;
}

static struct mb_struct * mb_search_mb(unsigned long phys)
{
	struct mba_host_struct *mbah = wmt_mbah;
	struct mb_area_struct *mba;
	struct mb_struct *mb;
	unsigned long flags;

	if(!phys){
		MB_WARN("mb_search_mb unknow addr %lx\n",phys);
		return NULL;
	}

	MB_DBG("IN, addr 0x%lx\n",phys);

	spin_lock_irqsave(&mb_search_lock, flags);

	list_for_each_entry(mba, &mbah->mba_list, mba_list){
		unsigned long pfn = phys >> PAGE_SHIFT;
		// out of range
		if((pfn < mba->pgi.pfn_start) || (pfn > mba->pgi.pfn_end))
			continue;

		list_for_each_entry(mb, &mba->mb_list, mb_list){
			if (mb->physical == phys){
				spin_unlock_irqrestore(&mb_search_lock, flags);
				return mb;
			}
		}
	}

	spin_unlock_irqrestore(&mb_search_lock, flags);

	MB_DBG("OUT, NULL mb\n");

	return NULL;
}

// addr - search address
// type - search type
//		  0 - virtual address
//		  1 - user address (task match required)
// way	MBUSRCH_ALL		search all mbu
//		MBUSRCH_CREATOR	search creator only
//		MBUSRCH_MMAP	search mbu which mapping from user
//						of course, skip owner
//		MBUSRCH_GETPUT	mbu came from get/put while mbusearch,
//						of course, skip owner
static struct mb_user * mb_search_mbu(
	unsigned long addr,
	unsigned int type,
	unsigned int way,
	pid_t tgid
)
{
	struct mba_host_struct *mbah = wmt_mbah;
	struct mb_area_struct *mba;
	struct mb_struct *mb;
	struct mb_user *mbu = NULL;
	unsigned long flags,phys = 0;

	if(!addr){
		MB_WARN("mb_search_mbu unknow addr %lx\n",addr);
		return NULL;
	}

	if(!type) // virtual to physical
		phys = __pa(addr);

	MB_DBG("IN, addr 0x%lx(0x%lx) type %d way %d TGID %d CurTask %s TGID %d\n",
		addr,phys,type,way,tgid,current->comm,current->tgid);

	spin_lock_irqsave(&mb_search_lock, flags);
	list_for_each_entry(mba, &mbah->mba_list, mba_list){
		if(!type){
			unsigned long pfn = phys >> PAGE_SHIFT;
			// out of range
			if((pfn < mba->pgi.pfn_start) || (pfn > mba->pgi.pfn_end))
				continue;
		}
		list_for_each_entry(mb, &mba->mb_list, mb_list){
			if(way == MBUSRCH_CREATOR){
				mbu = mb->creator;
				if (!mbu || mbu->addr != addr ||
					mbu->type != type)
					continue;
				if (mbu->tgid != tgid){
					MB_DBG("unmatch task, search TGID %d allocate TGID %d CurTask %s TGID %d",
						tgid,mbu->tgid,current->comm,current->tgid);
					continue;
				}
				goto leave;
			}
			list_for_each_entry(mbu, &mb->mbu_list, mbu_list){
				if(mbu->addr == addr && mbu->type == type && mbu->tgid == tgid){
					switch(way){
						case MBUSRCH_MMAP:
							if(!mbu->owner && mbu->size)	goto leave;
							break;
						case MBUSRCH_GETPUT:
							if(!mbu->owner && !mbu->size)	goto leave;
							break;
						case MBUSRCH_ALL:
							goto leave;
						default:
							MB_WARN("search mbu with unknow way.\n");
							mbu = NULL;
							goto leave;
					}
				}
			}
		}
	}

    mbu = NULL;

leave:

	if(mbu){
		MB_DBG("OUT, mbu %p(mb %p type %d owner %d TGID %d addr 0x%lx size %lx user %s)\n",
			mbu,mbu->mb,mbu->type,mbu->owner,mbu->tgid,mbu->addr,mbu->size,mbu->the_user);
	}
	else{
		MB_DBG("OUT, NULL mbu\n");
	}

	spin_unlock_irqrestore(&mb_search_lock, flags);

	return mbu;
}

int mb_do_counter(
	unsigned long addr,
	unsigned int type,
	char *name
)
{
	struct mb_struct *mb;
	unsigned long flags;
	int counter = -1;

	MB_DBG("IN, addr 0x%lx type %x name %s\n",addr,type,name);

	if(!addr || !name || !wmt_mbah){
		MB_WARN("mb_do_counter invalid paramaters %p/%lx/%s\n",wmt_mbah,addr,name);
		return -EINVAL;
	}
	spin_lock_irqsave(&mb_do_lock, flags);
	mb = mb_search_mb(addr);
	counter = (mb)?mb->count.counter:-1;
	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("OUT, addr 0x%lx count %d\n",addr,counter);

	return counter;
}

unsigned long mb_do_user_to_phys(
	unsigned long user,
	pid_t tgid
)
{
	struct mb_user *mbu;
	unsigned long flags,phys = 0x0;

	MB_DBG("IN, TGID %d task %s TGID %d usr 0x%lx\n",tgid,current->comm,current->tgid,user);

	spin_lock_irqsave(&mb_do_lock, flags);
	mbu = mb_search_mbu(user,1,MBUSRCH_ALL,tgid);
	if(!mbu){
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_WARN("mb_do_user_to_phys unknow addr %lx\n",user);
		return 0;
	}

	if(!mbu->mb)
		MB_WARN("unknow memory block.\n");
	else
		phys = mbu->mb->physical;

	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("OUT, phys 0x%lx\n",phys);

	return phys;
}

unsigned long mb_do_user_to_virt(
	unsigned long user,
	pid_t tgid
)
{
	unsigned long phys = mb_do_user_to_phys(user,tgid);

	MB_DBG("usr 0x%lx virt %p\n",user,__va(phys));

	if(!phys)
		return phys;

	return (unsigned long)__va(phys);
}

unsigned long mb_do_allocate(
	unsigned long size,
	unsigned int type,
	char *name,
	pid_t tgid
)
{
	struct mb_area_struct *entry,*mba = NULL;
	struct mb_struct *mb = NULL;
	struct mb_user *mbu = NULL;
	unsigned int pages;
	unsigned long flags,addr;

	if(type > 1 || !name || (type && tgid == MB_DEF_TGID)){
		MB_WARN("mb_allocate null user name or unknow type %d\n",type);
		return 0;
	}

	size = PAGE_ALIGN(size);
	pages = size >> PAGE_SHIFT;

	if(!pages){
		MB_WARN("mb_allocate unavailable size %x\n",pages);
		return 0;
	}

	mbu = kmem_cache_alloc(wmt_mbah->mbu_cachep, GFP_KERNEL);
	if(!mbu){
		MB_WARN("mbu_cachep out of memory.\n");
		return 0;
	}
	memset(mbu,0x0,sizeof(struct mb_user));

	MB_DBG("IN, TGID %d task %s TGID %d size %lx tp %x name %s\n",
		tgid,current->comm,current->tgid,size,type,name);

	spin_lock_irqsave(&mb_do_lock, flags);

	if(pages > wmt_mbah->max_available_pages){
		if(!(mba = mb_allocate_mba(pages)))
			MB_WARN("mb_allocate create MBA fail, tot %u/%lu/%lu/%lu mba %d\n",
				pages,wmt_mbah->max_available_pages,wmt_mbah->tot_free_pages,
				wmt_mbah->tot_pages,wmt_mbah->nr_mba);
	}
	else{
		list_for_each_entry(entry, &wmt_mbah->mba_list, mba_list){
			if(entry->max_available_pages >= pages){
				mba = entry;
				break;
			}
		}
	}

	if(!mba){
		MB_WARN("mb_allocate dedicated MBA not found\n");
		goto error;
	}

	if(!(mb = mb_allocate_mb(mba, size))){
		MB_WARN("mb_allocate create MB fail\n");
		goto error;
	}

	INIT_LIST_HEAD(&mbu->mbu_list);
	mbu->owner = 1;
	mbu->type = type;
	mbu->mb = mb;
	mbu->tgid = tgid;
	mbu->size = mb->size;
	mbu->addr = (unsigned long)__va(mb->physical);
	if(type) // allocate from user, get user space address from mmap
		mbu->addr = mb->physical;
	strncpy(mbu->the_user, name, TASK_COMM_LEN);
	list_add_tail( &mbu->mbu_list, &mb->mbu_list);
	atomic_inc(&mb->count);
	mb->creator = mbu;
	addr = mbu->addr;
	spin_unlock_irqrestore(&mb_do_lock, flags);
	return addr;

error:

	kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
	spin_unlock_irqrestore(&mb_do_lock, flags);
	return 0;

}

int mb_do_free(
	unsigned long addr,
	unsigned int type,
	char *name,
	pid_t tgid
)
{
	struct mb_struct *mb;
	struct mb_user *mbu;
	unsigned long flags;
	int ret;

	if(!addr || !name){
		MB_WARN("mb_free invalid paramaters %lx/%s\n",addr,name);
		return -EINVAL;
	}

	MB_DBG("IN, TGID %d task %s TGID %d addr 0x%lx type %x name %s\n",
		tgid,current->comm,current->tgid,addr,type,name);

	spin_lock_irqsave(&mb_do_lock, flags);

	mbu = mb_search_mbu(addr,type,MBUSRCH_CREATOR,tgid);
	if(!mbu || !mbu->mb){
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_WARN("mb_free unknow addr %lx\n",addr);
		return -EINVAL;
	}

	if(strncmp(mbu->the_user,name,TASK_COMM_LEN)){
		MB_DBG("Owner no match. (%s/%s)\n", mbu->the_user, name);
	}

	mb = mbu->mb;

	list_del(&mbu->mbu_list);
	kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
	mb->creator = NULL;
	atomic_dec(&mb->count);
	if(MB_IN_USE(mb)){
		MB_DBG("<mb_free, MB %8lx still in use.>\n",mb->physical);
		spin_unlock_irqrestore(&mb_do_lock, flags);
		return 0;
	}

	ret = mb_free_mb(mb);
	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("OUT\n");

	return ret;
}

int mb_do_get(
	unsigned long addr,
	unsigned int type,
	char *name,
	pid_t tgid
)
{
	struct mb_struct *mb;
	struct mb_user *mbu,*newmbu;
	unsigned long flags;

	if(!addr || !name){
		MB_WARN("mb_do_get invalid paramaters %lx/%s\n",addr,name);
		return -EINVAL;
	}

	newmbu = kmem_cache_alloc(wmt_mbah->mbu_cachep, GFP_KERNEL);
	if(!newmbu){
		MB_DBG("<mbu_cachep out of memory.>\n");
		return -ENOMEM;
	}

	MB_DBG("IN, TGID %d task %s TGID %d addr 0x%lx type %x name %s\n",
		tgid,current->comm,current->tgid,addr,type,name);

	spin_lock_irqsave(&mb_do_lock, flags);
	if(type){
		mbu = mb_search_mbu(addr,type,MBUSRCH_ALL,tgid);
		if(!mbu){
			spin_unlock_irqrestore(&mb_do_lock, flags);
			kmem_cache_free(wmt_mbah->mbu_cachep, newmbu);
			MB_WARN("mb_get unknow user addr %8lx\n",addr);
			return -EFAULT;
		}
		mb = mbu->mb;
	}
	else
		mb = mb_search_mb(__pa(addr));

	if(!mb){
		spin_unlock_irqrestore(&mb_do_lock, flags);
		kmem_cache_free(wmt_mbah->mbu_cachep, newmbu);
		//MB_WARN("mb_get unknown mb addr %8lx\n",addr);
		return -EFAULT;
	}

	memset(newmbu,0x0,sizeof(struct mb_user));
	INIT_LIST_HEAD(&newmbu->mbu_list);
	newmbu->addr = addr;
	newmbu->type = type;
	newmbu->mb = mb;
	newmbu->tgid = tgid; // if come from kernel function call, TGID should be default
	strncpy(newmbu->the_user, name, TASK_COMM_LEN);
	list_add_tail( &newmbu->mbu_list, &mb->mbu_list);

	atomic_inc(&mb->count);

	MB_DBG("out, [mbu] addr %lx %s [mb] addr %lx cnt %d\n",
		addr,newmbu->the_user,mb->physical,mb->count.counter);

	spin_unlock_irqrestore(&mb_do_lock, flags);

	return 0;
}

int mb_do_put(
	unsigned long addr,
	unsigned int type,
	char *name,
	pid_t tgid
)
{
	struct mb_struct *mb;
	struct mb_user *mbu;
	unsigned long flags;

	if(!addr || !name){
		MB_WARN("mb_do_put invalid paramaters %lx/%s\n",addr,name);
		return -EINVAL;
	}

	MB_DBG("IN, TGID %d task %s TGID %d addr 0x%lx size %x name %s\n",
		tgid,current->comm,current->tgid,addr,type,name);

	spin_lock_irqsave(&mb_do_lock, flags);
	mbu = mb_search_mbu(addr,type,MBUSRCH_GETPUT,tgid);
	if(!mbu){
		spin_unlock_irqrestore(&mb_do_lock, flags);
		//MB_WARN("mb_put unknow virt %8lx\n",addr);
		return -EINVAL;
	}

	mb = mbu->mb;
	if(!mb->count.counter){
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_WARN("memory block %p unbalance.\n",mb);
		return -EPERM;
	}

	list_del_init(&mbu->mbu_list);
	atomic_dec(&mb->count);
	// retrieve unused memory block
	if(!MB_IN_USE(mb))
		mb_free_mb(mb);

	MB_DBG("out, [mbu] addr %lx %s [mb] addr %lx cnt %d\n",
		addr,mbu->the_user,mb->physical,mb->count.counter - 1);

	spin_unlock_irqrestore(&mb_do_lock, flags);

	kmem_cache_free(wmt_mbah->mbu_cachep, mbu);

	return 0;
}

#define DEVICE_NAME "Memory Block"

static int mb_dev_major = 216;
static int mb_dev_minor = 0;
static int mb_dev_nr = 1;
static struct cdev *mb_cdev = NULL;

static int mb_open(struct inode *inode, struct file *filp)
{
	struct task_struct *task;
	filp->private_data = current;
	task = (struct task_struct*)(filp->private_data);
	MB_INFO("mb driver is opened by task %s TGID %d.\n",
		(task)?task->comm:"unknown",(task)?task->tgid:MB_DEF_TGID);
	return (task)?0:-EFAULT;
}

static int mb_release(struct inode *inode, struct file *filp)
{
	struct task_struct *task;
	struct mb_area_struct *mba;
	struct mb_struct *mb;
	struct mb_user *mbu;
	unsigned long flags;

	task = (struct task_struct*)(filp->private_data);

	MB_INFO("mb driver is closed by task %s TGID %d.\n",
		(task)?task->comm:"unknown",(task)?task->tgid:MB_DEF_TGID);

	spin_lock_irqsave(&mb_ioctl_lock, flags);

RESTART:
	// munmap virtual memroy and retrieve unused MB
	list_for_each_entry(mba, &wmt_mbah->mba_list, mba_list){
		// free prefetched mba marker
		if(mba->tgid == task->tgid){
			wmt_mbah->tot_static_pages -= mba->pages;
			mba->flags &= ~(MBAFLAG_STATIC);
			mba->tgid = MB_DEF_TGID;
			if(!mba->nr_mb){
				mb_free_mba(mba);
				goto RESTART; // ugly jump bcz I am so lazy to restructure it
			}
		}
		list_for_each_entry(mb, &mba->mb_list, mb_list){
			list_for_each_entry(mbu, &mb->mbu_list, mbu_list){
				if(mbu->type && mbu->tgid == task->tgid){
					MB_WARN("[%s] mb addr %lx doesn't %s\n",
						mbu->the_user,mbu->addr,
						(mbu->size)?"ummap":"put back");
					if(mbu->owner)
						mb->creator = NULL;
					list_del_init(&mbu->mbu_list);
					kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
					atomic_dec(&mb->count);
					break;
				}
			}
			if(!MB_IN_USE(mb)){
				mb_free_mb(mb);
				// ugly jump because mba link maybe breaken and
				// 	I am so lazy to restructure it
				goto RESTART;
			}
		}
	}

	mb_update_mbah();

	spin_unlock_irqrestore(&mb_ioctl_lock, flags);

	return 0;
}

static int mb_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct task_struct *task;
	struct mb_struct *mb = NULL;
	struct mb_user *mbu = NULL;
	unsigned long value,flags,phys,virt,size;
	pid_t tgid = MB_DEF_TGID;
	int ret = 0;

	/* check type and number, if fail return ENOTTY */
	if( _IOC_TYPE(cmd) != MB_IOC_MAGIC )	return -ENOTTY;

	/* check argument area */
	if( _IOC_DIR(cmd) & _IOC_READ )
		ret = !access_ok( VERIFY_WRITE, (void __user *) arg, _IOC_SIZE(cmd));
	else if ( _IOC_DIR(cmd) & _IOC_WRITE )
		ret = !access_ok( VERIFY_READ, (void __user *) arg, _IOC_SIZE(cmd));

	if( ret )	return -EFAULT;

	/* check task validation */
	if(!filp || !filp->private_data){
		MB_ERROR("mmap with null file info or task.\n");
		return -EFAULT;
	}

	task = (struct task_struct*)(filp->private_data);
	tgid = task->tgid;
	if(tgid != current->tgid)
		MB_WARN("ioctl btw diff tasks Open: task %s TGID %d Curr: task %s TGID %d.\n",
			task->comm,tgid,current->comm,current->tgid);

	switch(cmd){
		// _IOWR(MB_IOC_MAGIC, 0, unsigned long)
		// 		I: size
		//		O: physical address
		case MBIO_MALLOC:
			get_user(value, (unsigned long *)arg);

			phys = mb_do_allocate(value,1,task->comm,tgid);
			if(!phys){
				MB_WARN("MBIO_MALLOC size %lx fail.\n",value);
				return -EFAULT;
			}

			MB_IOCTLDBG("<MBIO_MALLOC addr %lx size %lx>\n",phys,value);
			put_user(phys, (unsigned long *)arg);
			break;

		// _IOWR(MB_IOC_MAGIC, 1, unsigned long)
		// 		I: user address if map, physical address if not map
		//		O: ummap size
		case MBIO_FREE:
			get_user(value, (unsigned long *)arg);

			spin_lock_irqsave(&mb_ioctl_lock, flags);
			mbu = mb_search_mbu(value,1,MBUSRCH_CREATOR,tgid);
			if(!mbu || !mbu->mb){
				spin_unlock_irqrestore(&mb_ioctl_lock, flags);
				MB_WARN("MBIO_FREE user %lx fail. (addr not found)\n",value);
				return -EFAULT;
			}
			phys = mbu->mb->physical;
			size = mbu->size;
			mbu->addr = phys;
			mbu->size = 0;
			ret = mb_do_free(phys,1,task->comm,tgid);
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			if(ret < 0)
				MB_WARN("MBIO_FREE user %lx fail.\n",value);
			put_user(size, (unsigned long *)arg);	// return mmap size for ummap
			MB_IOCTLDBG("<MBIO_FREE addr %lx/%lx size %lx>\n",value,phys,size);
			break;

		// _IOWR(MB_IOC_MAGIC, 2, unsigned long)
		// 		I: user address
		//		O: ummap size
		case MBIO_UNMAP:
			get_user(value, (unsigned long *)arg);

			spin_lock_irqsave(&mb_ioctl_lock, flags);
			mbu = mb_search_mbu(value,1,MBUSRCH_MMAP,tgid);
			if(!mbu){
				spin_unlock_irqrestore(&mb_ioctl_lock, flags);
				MB_WARN("MBIO_UNMAP user %lx fail. (addr not found)\n",value);
				return -EFAULT;
			}

			phys = (mbu && mbu->mb)?mbu->mb->physical:0x0;
			size = mbu->size;
			list_del(&mbu->mbu_list);
			kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
			atomic_dec(&mb->count);
			if(MB_IN_USE(mb)){
				spin_unlock_irqrestore(&mb_ioctl_lock, flags);
				put_user(size, (unsigned long *)arg);	// return mmap size for ummap
				MB_DBG("<MBIO_UNMAP, MB %8lx still in use.>\n",mb->physical);
				return 0;
			}
			ret = mb_free_mb(mb);
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			MB_IOCTLDBG("<MBIO_UNMAP addr %lx/%lx size %lx>\n",value,phys,size);
			put_user(size, (unsigned long *)arg);	// return mmap size for ummap
			break;

		// _IOWR(MB_IOC_MAGIC, 3, unsigned long)
		// 		I: phys address
		//		O: mb size
		case MBIO_MBSIZE:
			get_user(value, (unsigned long *)arg);
			spin_lock_irqsave(&mb_ioctl_lock, flags);
			mb = mb_search_mb(value);
			if(!mb){
				spin_unlock_irqrestore(&mb_ioctl_lock, flags);
				MB_WARN("MBIO_UNMAP user %lx fail. (addr not found)\n",value);
				return -EFAULT;
			}
			size = (mb)?mb->size:0;
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			MB_IOCTLDBG("<MBIO_MBSIZE addr %lx size %lx >\n",value,size);
			put_user(size, (unsigned long *)arg);	// return mb size
			break;

		// _IOR (MB_IOC_MAGIC, 4, unsigned long)
		// 		O: max free mba size
		case MBIO_MAX_AVAILABLE_SIZE:
			put_user(wmt_mbah->max_available_pages*PAGE_SIZE, (unsigned long *)arg);
			MB_IOCTLDBG("<MBIO_MAX_MB_SIZE. size %lx kb>\n",
				PAGE_KB(wmt_mbah->max_available_pages));
			break;

		// _IOW (MB_IOC_MAGIC, 5, unsigned long)
		// 		I: user address
		case MBIO_GET:
			get_user(value, (unsigned long *)arg);
			ret = mb_do_get(value,1,task->comm,tgid);
			if(MBMSG_LEVEL){
				spin_lock_irqsave(&mb_ioctl_lock, flags);
				mbu = mb_search_mbu(value,1,MBUSRCH_GETPUT,tgid);
				MB_IOCTLDBG("<MBIO_GET. addr %lx cnt %x>\n",
					value,mbu?mbu->mb->count.counter:0xffffffff);
				spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			}
			break;

		// _IOW (MB_IOC_MAGIC, 6, unsigned long)
		// 		I: user address
		case MBIO_PUT:
			get_user(value, (unsigned long *)arg);
			if(MBMSG_LEVEL){
				spin_lock_irqsave(&mb_ioctl_lock, flags);
				mbu = mb_search_mbu(value,1,MBUSRCH_GETPUT,tgid);
				MB_IOCTLDBG("<MBIO_PUT. virt %lx cnt %x>\n",
					value,mbu?(mbu->mb->count.counter - 1):0xffffffff);
				spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			}
			ret = mb_do_put(value,1,current->comm,tgid);
			break;

		// _IOWR(MB_IOC_MAGIC, 7, unsigned long)
		// 		I: user address
		//		O: virt address
		case MBIO_USER_TO_VIRT:
			get_user(value, (unsigned long *)arg);
			virt = mb_do_user_to_virt(value,tgid);
			put_user(virt, (unsigned long *)arg);
			MB_IOCTLDBG("<MBIO_USER_TO_VERT. user %8lx virt %lx>\n",
				value,virt);
			break;

		// _IOWR(MB_IOC_MAGIC, 8, unsigned long)
		// 		I: user address
		//		O: phys address
		case MBIO_USER_TO_PHYS:
			get_user(value, (unsigned long *)arg);
			phys = mb_do_user_to_phys(value,tgid);
			put_user(phys, (unsigned long *)arg);
			MB_IOCTLDBG("<MBIO_USER_TO_PHYS. user %8lx phys %lx>\n",
				value,phys);
			break;

		// _IOW (MB_IOC_MAGIC, 9, unsigned long)
		// 		I: size
		case MBIO_PREFETCH:
		{
			unsigned long try_size = 0;
			unsigned long fetch_size = 0;
			struct mb_area_struct *mba;

			get_user(value, (unsigned long *)arg);
			spin_lock_irqsave(&mb_ioctl_lock, flags);
			while(try_size < value){
				int order;
				order = get_order(value-fetch_size);
				order = min(order,(int)MBMAX_ORDER);
				try_size += (1<<order)*PAGE_SIZE;
				mba = mb_allocate_mba(1 << order);
				if(mba){
					mba->flags |= MBAFLAG_STATIC;
					mba->tgid = current->tgid;
					wmt_mbah->tot_static_pages += mba->pages;
					fetch_size += mba->pages * PAGE_SIZE;
				}
			}
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			MB_INFO("Pre-fetch BUFFER for %s (SIZE %ld / %ld kB)\n",
				current->comm,fetch_size/1024,value/1024);
			break;
		}

		// _IOW (MB_IOC_MAGIC,10, unsigned long)
		// 		O: static mba size
		case MBIO_STATIC_SIZE:
			put_user(wmt_mbah->tot_static_pages*PAGE_SIZE, (unsigned long *)arg);
			MB_IOCTLDBG("<MBIO_STATIC_SIZE. size %lx kb>\n",
				PAGE_KB(wmt_mbah->tot_static_pages));
			break;

		// _IOWR(MB_IOC_MAGIC,11, unsigned long)
		// 		I: physical address
		//		O: use counter
		case MBIO_MB_USER_COUNT:
			get_user(value, (unsigned long *)arg);

			spin_lock_irqsave(&mb_ioctl_lock, flags);
			mb = mb_search_mb(value);
			if(mb == NULL)
				size = 0;
			else
				size = mb->count.counter;
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);

			put_user(size, (unsigned long *)arg);
			MB_IOCTLDBG("<MBIO_MB_USER_COUNT. addr %lx count %ld>\n",value,size);
			break;

		// _IO  (MB_IOC_MAGIC,12)
		case MBIO_FORCE_RESET:
		{
			struct mb_area_struct *mba;
			spin_lock_irqsave(&mb_ioctl_lock, flags);
RESTART:
			list_for_each_entry(mba, &wmt_mbah->mba_list, mba_list){
				list_for_each_entry(mb, &mba->mb_list, mb_list){
					list_for_each_entry(mbu, &mb->mbu_list, mbu_list){
						list_del(&mbu->mbu_list);
						kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
						atomic_dec(&mb->count);
					}
					mb->creator = NULL;
					atomic_set(&mb->count, 0);
					mb_free_mb(mb);
					goto RESTART;	// because mba link maybe breaken
				}
			}
			mb_update_mbah();
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			MB_IOCTLDBG("<MBIO_FORCE_RESET OK.>\n");
			break;
		}

#ifdef MB_UNITTEST
		case MBIO_PRDT_LOOKBACK: // _IOW (MB_IOC_MAGIC,11,struct mem_area_struct)
		{
			struct mem_area_struct ma;
			struct prdt_struct *prdt;
			void *iv,*ov;

			copy_from_user(&ma, (const void *)arg, sizeof(struct mem_area_struct));

			iv = user_to_virt(ma.input);
			ov = user_to_virt(ma.output);
			MB_IOCTLDBG("MBIO_PRDT_LOOKBACK: cp from %lx/%p/%lx to %lx/%p/%lx\n",
				ma.input,iv,ma.isize,ma.output,ov,ma.osize);

			prdt = (struct prdt_struct *)kmalloc(
					((ma.isize/PAGE_SIZE) + 2) * sizeof(struct prdt_struct),
					GFP_KERNEL);

			if(!prdt){
				ret = -ENOMEM;
				break;
			}

			if(!user_to_prdt(ma.input,ma.isize,prdt,(ma.isize/PAGE_SIZE) + 2)){
				unsigned long size = 0;
				struct prdt_struct *pprdt = prdt;

				while(size < ma.osize){
					memcpy((void *)(ov + size),(void *)(__va(pprdt->addr)),pprdt->size);
					if(pprdt->EDT)
						break;
					size +=  pprdt->size;
					pprdt++;
				}
			}
			else{
				MB_DBG("<USER to PRD fail.>\n");
			}
			kfree(prdt);
			break;
		}
#endif

		default:
			ret = -ENOTTY;
			break;
	}

	return ret;
}

static int mb_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long phys,flags;
	unsigned int len;
	struct mb_struct *mb;
	struct mb_user *mbu,*new_mbu;
	struct task_struct *task;
	pid_t tgid = MB_DEF_TGID;

	if(!filp || !filp->private_data){
		MB_ERROR("mmap with null file info or task.\n");
		return -EFAULT;
	}

	task = (struct task_struct*)(filp->private_data);
	tgid = task->tgid;
	if(tgid != current->tgid)
		MB_WARN("mmap btw diff tasks Open: task %s TGID %d Curr: task %s TGID %d.\n",
			task->comm,tgid,current->comm,current->tgid);

	new_mbu = kmem_cache_alloc(wmt_mbah->mbu_cachep, GFP_KERNEL);
	if(!new_mbu){
		MB_ERROR("mbu_cachep out of memory.\n");
		return -ENOMEM;
	}

	phys = vma->vm_pgoff << PAGE_SHIFT;

	spin_lock_irqsave(&mb_ioctl_lock, flags);
	mb = mb_search_mb(phys);
	if(!mb){
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		kmem_cache_free(wmt_mbah->mbu_cachep, new_mbu);
		MB_ERROR("map addr %lx not exist in MB.\n",phys);
		return -EINVAL;
	}

	len = PAGE_ALIGN(mb->physical + mb->size);
	if ((vma->vm_end - vma->vm_start + phys) > len){
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		kmem_cache_free(wmt_mbah->mbu_cachep, new_mbu);
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
					vma->vm_end - vma->vm_start, vma->vm_page_prot)){
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		kmem_cache_free(wmt_mbah->mbu_cachep, new_mbu);
		MB_ERROR("virtual address memory map fail.\n");
		return -EAGAIN;
	}

	// mb creator has been allocated from MBIO_MALLOC
	// but user address is not mapped yet.
	mbu = mb->creator;
	if(!mbu || mbu->tgid != tgid || mbu->addr != phys){
		mbu = new_mbu;
		memset(mbu,0x0,sizeof(struct mb_user));
		INIT_LIST_HEAD(&mbu->mbu_list);
		mbu->type = 1;
		mbu->tgid = tgid;
		mbu->mb = mb;
		strncpy(mbu->the_user, task->comm, TASK_COMM_LEN);
		list_add_tail( &mbu->mbu_list, &mb->mbu_list);
		atomic_inc(&mb->count);
	}

	mbu->addr = vma->vm_start;
	mbu->size = vma->vm_end - vma->vm_start;
	phys = mb->physical;
	spin_unlock_irqrestore(&mb_ioctl_lock, flags);

	MB_IOCTLDBG("<MB_MAP from %lx to %lx/%lx size %lx>\n",
		phys,vma->vm_start,vma->vm_end,(vma->vm_end - vma->vm_start));

	return 0;
}

/*!*************************************************************************
	driver file operations struct define
****************************************************************************/
struct file_operations mb_fops = {
	.owner = THIS_MODULE,
	.open = mb_open,
	.release = mb_release,
	.ioctl = mb_ioctl,
	.mmap = mb_mmap,
};

static int mb_probe(struct platform_device *dev)
{
	struct mb_area_struct *mba = NULL, *bind = NULL;
	unsigned long flags;
	int ret = 0;
	dev_t dev_no;

	dev_no = MKDEV(mb_dev_major,mb_dev_minor);

	if(wmt_mbah){
		MB_ERROR("%s dirty.\n",DEVICE_NAME);
		return -EINVAL;
	}

	/* register char device */
	mb_cdev = cdev_alloc();
	if(!mb_cdev){
		MB_ERROR("alloc dev error.\n");
		return -ENOMEM;
	}

	cdev_init(mb_cdev,&mb_fops);
	ret = cdev_add(mb_cdev,dev_no,1);

	if(ret){
		MB_ERROR("reg char dev error(%d).\n",ret);
		cdev_del(mb_cdev);
		return ret;
	}

	wmt_mbah = (struct mba_host_struct *)kmalloc(sizeof(struct mba_host_struct), GFP_KERNEL);
	if(!wmt_mbah){
		MB_ERROR("out of memory (mb_area_struct).\n");
		cdev_del(mb_cdev);
		return -ENOMEM;
	}
	memset(wmt_mbah, 0x0, sizeof(struct mba_host_struct));

	INIT_LIST_HEAD(&wmt_mbah->mba_list);
	wmt_mbah->mba_cachep = kmem_cache_create( "mb_area_struct",
											 sizeof(struct mb_area_struct),
											 0,
											 SLAB_HWCACHE_ALIGN,
											 NULL);
	if(!wmt_mbah->mba_cachep){
		MB_ERROR("out of memory (mba_cachep).\n");
		kfree(wmt_mbah);
		cdev_del(mb_cdev);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&wmt_mbah->mba_list);
	wmt_mbah->mb_cachep = kmem_cache_create( "mb_struct",
											sizeof(struct mb_struct),
											0,
											SLAB_HWCACHE_ALIGN,
											NULL);
	if(!wmt_mbah->mb_cachep){
		MB_ERROR("out of memory (mb_cachep).\n");
		kmem_cache_destroy(wmt_mbah->mba_cachep);
		kfree(wmt_mbah);
		cdev_del(mb_cdev);
		return -ENOMEM;
	}

	wmt_mbah->mbu_cachep = kmem_cache_create( "mb_user",
											sizeof(struct mb_user),
											0,
											SLAB_HWCACHE_ALIGN,
											NULL);
	if(!wmt_mbah->mbu_cachep){
		MB_ERROR("out of memory (mbu_cachep).\n");
		kmem_cache_destroy(wmt_mbah->mb_cachep);
		kmem_cache_destroy(wmt_mbah->mba_cachep);
		kfree(wmt_mbah);
		cdev_del(mb_cdev);
		return -ENOMEM;
	}

	MBMAX_ORDER = MBA_MAX_ORDER;
	MBMIN_ORDER = MBA_MIN_ORDER;

    spin_lock_init(&mb_do_lock);
    spin_lock_init(&mb_search_lock);
    spin_lock_init(&mb_ioctl_lock);
    spin_lock_init(&mb_task_mm_lock);

	MB_INFO("Preparing VIDEO BUFFER (SIZE %d kB) ...\n",CONFIG_VIDEO_BUFFER_SIZE);
	spin_lock_irqsave(&mb_do_lock,flags);
	while(PAGE_KB(wmt_mbah->tot_static_pages) < CONFIG_VIDEO_BUFFER_SIZE){
		int order;
		order = get_order(	CONFIG_VIDEO_BUFFER_SIZE*1024 -
							wmt_mbah->tot_static_pages*PAGE_SIZE);
		order = min(order,(int)MBMAX_ORDER);
		mba = mb_allocate_mba(1 << order);
		if(mba){
			mba->flags |= MBAFLAG_STATIC;
			mba->tgid = MB_DEF_TGID;
			wmt_mbah->tot_static_pages += mba->pages;
			// conbine to continue mba if possible
			if(bind && bind->pgi.pfn_start == mba->pgi.pfn_end){
				MB_COMBIND_MBA(mba,bind);
				bind = mba;
			}
			else if(bind && bind->pgi.pfn_end == mba->pgi.pfn_start){
				MB_COMBIND_MBA(bind,mba);
			}
			else
				bind = mba;
		}
	}
	spin_unlock_irqrestore(&mb_do_lock,flags);

	mb_update_mbah();
	MB_INFO("MAX MB Area size: Max %ld Kbs Min %ld Kbs\n",
		PAGE_KB(1 << MBMAX_ORDER),PAGE_KB(1 << MBMIN_ORDER));

	MB_INFO("prob /dev/%s major %d, minor %d\n",
		DEVICE_NAME, mb_dev_major,mb_dev_minor);

	return ret;
}

static int mb_remove(struct platform_device *dev)
{
	if(mb_cdev)
		cdev_del(mb_cdev);

	if(wmt_mbah){
		if(wmt_mbah->mba_cachep)
			kmem_cache_destroy(wmt_mbah->mba_cachep);
		if(wmt_mbah->mb_cachep)
			kmem_cache_destroy(wmt_mbah->mb_cachep);
		if(wmt_mbah->mbu_cachep)
			kmem_cache_destroy(wmt_mbah->mbu_cachep);
		kfree(wmt_mbah);
	}

	MB_INFO("MB dev remove \n");
	return 0;
}

static int mb_suspend(struct platform_device *dev, pm_message_t state)
{
	MB_DBG("MB get suspend, Event %d.\n",state.event);
	return 0;
}

static int mb_resume(struct platform_device *dev)
{
	MB_DBG("MB get resume.\n");
	return 0;
}

int mb_area_read_proc(
	char *page, char **start, off_t off,
	int count, int *eof, void *data
)
{
	struct mb_area_struct *mba;
	struct free_area *fa;
	struct zone *zone;
	unsigned long flags;
	unsigned int idx = 1;
    char *p = page;
	int len;

	spin_lock_irqsave(&mb_ioctl_lock, flags);

	if(!wmt_mbah){
		p += sprintf(p, "no MB area host existed.\n");
		goto leave;
	}

	p += sprintf(p,"\n");
	p += sprintf(p,"MESSAGE LEVEL:    %8d   /%8lx\n",MBMSG_LEVEL,__pa(&MBMSG_LEVEL));
	p += sprintf(p,"STATIC MB SIZE:  %8ld kB /%8d kB\n",
				PAGE_KB(wmt_mbah->tot_static_pages),CONFIG_VIDEO_BUFFER_SIZE);
	p += sprintf(p,"MAX MBA ORDER:    %8d   /%8lx\n",
				MBMAX_ORDER,__pa(&MBMAX_ORDER));
	p += sprintf(p,"MIN MBA ORDER:    %8d   /%8lx\n\n",
				MBMIN_ORDER,__pa(&MBMIN_ORDER));
	p += sprintf(p,"total MB areas:  %8d\n",wmt_mbah->nr_mba);
	p += sprintf(p,"total size:      %8ld kB\n",PAGE_KB(wmt_mbah->tot_pages));
	p += sprintf(p,"total free size: %8ld kB\n",PAGE_KB(wmt_mbah->tot_free_pages));
	p += sprintf(p,"max MB size:     %8ld kB\n\n",PAGE_KB(wmt_mbah->max_available_pages));

	list_for_each_entry(mba, &wmt_mbah->mba_list, mba_list){
		p += sprintf(p, "(ID)         [MB Area]  address     size [  zs,  ze]"
						" MBs      Max/Free\n");
		len = (int)(p-page);
		if((count - (len - off)) < 10){
			p += sprintf(p,	" more ...\n");
			goto leave;
		}
		p += sprintf(p,"(%2d)",idx++);
		len = (int)(p-page);
		p += mb_show_mba(mba, NULL, p, count - (len - off), 1); // show all MBs
		p += sprintf(p,"\n");
	}

	// show memory fragment
	zone = (first_online_pgdat())->node_zones;
	fa = zone->free_area;
	p += sprintf( p, "DMA ZONE:\n");
	for(idx = 0; idx < MAX_ORDER; idx++){
		p += sprintf( p, "         %5ld * %5ldkB    = %5ld kB\n",
					fa[idx].nr_free,PAGE_KB((1<<idx)),fa[idx].nr_free*PAGE_KB((1<<idx)));
	}
	p += sprintf( p, "         ------------------------------\n");
	p += sprintf( p, "                          + = %ld kB\n",PAGE_KB((unsigned long)nr_free_pages()));

leave:

	spin_unlock_irqrestore(&mb_ioctl_lock, flags);

	len = (int)(p-page);
	if(len <= off+count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if(len > count)
		len = count;
	if(len <  0)
		len = 0;

    return len;
}

static struct platform_driver mb_driver = {
	.driver.name           = "wmt_mb", // This name should equal to platform device name.
	.probe          = mb_probe,
	.remove         = mb_remove,
	.suspend        = mb_suspend,
	.resume         = mb_resume
};

static void mb_platform_release(struct device *device)
{
	return;
}

static struct platform_device mb_device = {
	.name           = "wmt_mb",
	.id             = 0,
	.dev            = 	{	.release =  mb_platform_release,
#if 0
							.dma_mask = &spi_dma_mask,
							.coherent_dma_mask = ~0,
#endif
						},
	.num_resources  = 0,		/* ARRAY_SIZE(spi_resources), */
	.resource       = NULL,		/* spi_resources, */
};

static int __init mb_init(void)
{
	int ret = -1;
	dev_t dev_no;
	static struct class *fc;

	if( !mb_dev_major ){
		MB_ERROR("unknow %s device.(%d)\n",DEVICE_NAME,mb_dev_major);
		return -ENODEV;
	}

	dev_no = MKDEV(mb_dev_major,mb_dev_minor);
	ret = register_chrdev_region(dev_no,mb_dev_nr,"wmt_mb");
	if( ret < 0 ){
		MB_ERROR("can't get %s device major %d\n",DEVICE_NAME,mb_dev_major);
		return ret;
	}

#ifdef CONFIG_PROC_FS
{
   // create_proc_info_entry("mbinfo", 0, NULL, mb_area_read_proc);
    struct proc_dir_entry *res=create_proc_entry("mbinfo", 0, NULL);
    if (res) {
        res->read_proc = mb_area_read_proc;
        MB_DBG("create MB proc\n");
    }
}
#endif

	ret = platform_driver_register(&mb_driver);
	if(!ret){
		ret = platform_device_register(&mb_device);
		if(ret)
			platform_driver_unregister(&mb_driver);
	}


	fc=class_create(THIS_MODULE, "mbdev");

	device_create(fc, NULL, dev_no, NULL, "%s","mbdev");
        printk("mb_init ok\n");
	return ret;
}

static void __exit mb_exit(void)
{
	dev_t dev_no;

	platform_driver_unregister(&mb_driver);
	platform_device_unregister(&mb_device);
	dev_no = MKDEV(mb_dev_major,mb_dev_minor);
	unregister_chrdev_region(dev_no,mb_dev_nr);

	return;
}

EXPORT_SYMBOL(user_to_virt);
EXPORT_SYMBOL(user_to_prdt);
EXPORT_SYMBOL(mb_do_user_to_virt);
EXPORT_SYMBOL(mb_do_user_to_phys);
EXPORT_SYMBOL(mb_do_allocate);
EXPORT_SYMBOL(mb_do_free);
EXPORT_SYMBOL(mb_do_put);
EXPORT_SYMBOL(mb_do_get);

subsys_initcall(mb_init);
module_exit(mb_exit);

MODULE_AUTHOR("WonderMedia Tech. SW Team Jason Lin");
MODULE_DESCRIPTION("memory block driver");
MODULE_LICENSE("GPL");
