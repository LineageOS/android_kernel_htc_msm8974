/*
 *  linux/fs/fat/move_cluster.c
 *
 *
 *  2014/07/09. move cluster, so that it used for defraging fragments
 *	 of fat32 files.
 */

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/buffer_head.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/netlink.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/dcache.h>
#include <linux/mount.h>
#include <linux/file.h>
#include <linux/fs_struct.h>
#include <linux/msdos_fs.h>
#include <linux/dirent.h>
#include <linux/stat.h>
#include <linux/falloc.h>
#include <linux/buffer_head.h>
#include <linux/namei.h>
#include <linux/fadvise.h>

#include <net/sock.h>

#include <asm/uaccess.h>
#include <asm/mach/arch.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>

#include "fat.h"







#define   FALLOC_FLAG   0xAA

/* 128kb is the whole sectors for FAT12 and FAT16 */
#define FAT_READA_SIZE		(128 * 1024)

#define PKG_COMMAND_FALLOCATE_FILE		1
#define PKG_COMMAND_FALLOCATE_DIR		2
#define PKG_COMMAND_IFCONTIGUOUS		3
#define PKG_COMMAND_GET_SUPERBLOCK		4
#define PKG_COMMAND_GET_BOOT_INFO		5
#define PKG_COMMAND_GET_FILE_INFO		6
#define PKG_COMMAND_GET_FREE_CLUSTER_INFO	7
#define PKG_COMMAND_GET_FILE_REF_COUNT		8
#define PKG_COMMAND_SET_NEW_HEAD		9
#define PKG_COMMAND_GET_CLUSTER_DATA		10
#define PKG_COMMAND_FADVISE			11
#define PKG_COMMAND_FALLOCATE_FILE_OLD		12
#define PKG_COMMAND_GET_CLUSTERS		13
#define PKG_COMMAND_MOVE_CLUSTERS		14



struct bootInfo
{
	u_int	BytesPerSec;		/* bytes per sector */
	u_int	SecPerClust;		/* sectors per cluster */
	u_int	ResSectors;		/* number of reserved sectors */
	u_int	FATs;			/* number of FATs */
	u_int	RootDirEnts;		/* number of root directory entries */
	u_int	Media;			/* media descriptor */
	u_int	FATsmall;		/* number of sectors per FAT */
	u_int	SecPerTrack;		/* sectors per track */
	u_int	Heads;			/* number of heads */
	u_int32_t Sectors;		/* total number of sectors */
	u_int32_t HiddenSecs;		/* # of hidden sectors */
	u_int32_t HugeSectors;		/* # of sectors if bpbSectors == 0 */
	u_int	FSInfo;			/* FSInfo sector */
	u_int	Backup;			/* Backup of Bootblocks */
	u_int32_t	RootCl;			/* Start of Root Directory */
	u_int32_t	FSFree;			/* Number of free clusters acc. FSInfo */
	u_int32_t	FSNext;			/* Next free cluster acc. FSInfo */

	/* and some more calculated values */
	u_int	flags;			/* some flags: */

	int	ValidFat;		/* valid fat if FAT32 non-mirrored */
	u_int32_t	ClustMask;		/* mask for entries in FAT */
	u_int32_t	NumClusters;		/* # of entries in a FAT */
	u_int32_t NumSectors;		/* how many sectors are there */
	u_int32_t FATsecs;		/* how many sectors are in FAT */
	u_int32_t NumFatEntries;	/* how many entries really are there */
	u_int	ClusterOffset;		/* at what sector would sector 0 start */
	u_int	ClusterSize;		/* Cluster size in bytes */

	/* Now some statistics: */
	u_int	NumFiles;		/* # of plain files */
	u_int	NumFree;		/* # of free clusters */
	u_int	NumBad;			/* # of bad clusters */

	unsigned long		BlockSize;
};

struct fileInfo
{
	u_int32_t lcn;
	u_int32_t realClusters;
	u_int32_t uncompressedClusters;
};

struct freeClusterInfo
{
	u_int32_t MinimumLcn;
	u_int32_t MinimumSize;
	u_int32_t BeginLcn;
	u_int32_t EndLcn;
};

struct fallocInfo
{
	int mode;
	loff_t offset;
	loff_t len;
	u_int32_t newLcn;
};

struct refInfo
{
	int f_count;
};

struct setNewHeadInfo
{
	u_int32_t newLcn;
};

struct getClusterDataInfo
{
	u_int32_t lcnHead;
	u_int32_t lcnNums;
	unsigned char *data;
};

struct getClustersInfo
{
	u_int32_t *lcnList;
	u_int32_t lcnNum;
};

struct moveCluster
{
	u_int32_t lcn;
	u_int32_t lcnPrev;
	u_int32_t lcnNext;
};

struct moveClustersInfo
{
	__u32 donor_fd;	/* donor file descriptor */
	__u32 orig_fd;	/* orig file descriptor */
	__u64 orig_start;	/* logical start offset in block for orig */
	__u64 donor_start;	/* logical start offset in block for donor */
	__u64 len;	/* block length to be moved */
	__u64 moved_len;	/* moved block length */

	struct moveCluster lcnOrig;
	struct moveCluster lcnDonor;
};



struct dataPkg
{
	char checkFlag;
	char fileName[500];
	char cmd;

	ino_t inodeNumber;
	dev_t mainDevNumber;
	dev_t minorDevNumber;
	void *data;
};

struct fatent_operations {
	void (*ent_blocknr)(struct super_block *, int, int *, sector_t *);
	void (*ent_set_ptr)(struct fat_entry *, int);
	int (*ent_bread)(struct super_block *, struct fat_entry *,
			 int, sector_t);
	int (*ent_get)(struct fat_entry *);
	void (*ent_put)(struct fat_entry *, int);
	int (*ent_next)(struct fat_entry *);
};


static inline int fat_ent_update_ptr_ext(struct super_block *sb,
				     struct fat_entry *fatent,
				     int offset, sector_t blocknr)
{
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct fatent_operations *ops = sbi->fatent_ops;
	struct buffer_head **bhs = fatent->bhs;

	/* Is this fatent's blocks including this entry? */
	if (!fatent->nr_bhs || bhs[0]->b_blocknr != blocknr)
		return 0;
	if (sbi->fat_bits == 12) {
		if ((offset + 1) < sb->s_blocksize) {
			/* This entry is on bhs[0]. */
			if (fatent->nr_bhs == 2) {
				brelse(bhs[1]);
				fatent->nr_bhs = 1;
			}
		} else {
			/* This entry needs the next block. */
			if (fatent->nr_bhs != 2)
				return 0;
			if (bhs[1]->b_blocknr != (blocknr + 1))
				return 0;
		}
	}
	ops->ent_set_ptr(fatent, offset);
	return 1;
}

static inline void lock_fat2(struct msdos_sb_info *sbi)
{
	mutex_lock(&sbi->fat_lock);
}

static inline void unlock_fat2(struct msdos_sb_info *sbi)
{
	mutex_unlock(&sbi->fat_lock);
}

static inline int fat_ent_read_block2(struct super_block *sb,
				     struct fat_entry *fatent)
{
	struct fatent_operations *ops = MSDOS_SB(sb)->fatent_ops;
	sector_t blocknr;
	int offset;

	fatent_brelse(fatent);
	ops->ent_blocknr(sb, fatent->entry, &offset, &blocknr);
	return ops->ent_bread(sb, fatent, offset, blocknr);
}

static inline int fat_ent_next2(struct msdos_sb_info *sbi,
			       struct fat_entry *fatent)
{
	if (sbi->fatent_ops->ent_next(fatent)) {
		if (fatent->entry < sbi->max_cluster)
			return 1;
	}
	return 0;
}

unsigned long fat_find_free_clusters(struct super_block *sb, unsigned long clusterSum, int *contiguous)
{
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct fatent_operations *ops = sbi->fatent_ops;
	struct fat_entry fatent;
	unsigned long reada_blocks, reada_mask, cur_block;
	int err = 0, free;


	unsigned long clusters = 0;
	unsigned long start = 0;

	*contiguous = 0;

	lock_fat2(sbi);

	if (sbi->free_clusters != -1 && sbi->free_clus_valid && sbi->free_clusters < clusterSum)
	{
		printk(KERN_INFO "fat_find_free_clusters failed, free_clusters:%d, free_clus_valid:%d\n",
			sbi->free_clusters, sbi->free_clus_valid);
		start = 0;
		goto out2;
	}

	reada_blocks = FAT_READA_SIZE >> sb->s_blocksize_bits;
	reada_mask = reada_blocks - 1;
	cur_block = 0;

	free = 0;
	fatent_init(&fatent);
	fatent_set_entry(&fatent, FAT_START_ENT);
	while (fatent.entry < sbi->max_cluster)
	{
		/* readahead of fat blocks */
		/*if ((cur_block & reada_mask) == 0)
		{
			unsigned long rest = sbi->fat_length - cur_block;
			fat_ent_reada2(sb, &fatent, min(reada_blocks, rest));
		}
		cur_block++;*/

		fatent_set_entry(&fatent, fatent.entry);

		err = fat_ent_read_block2(sb, &fatent);
		if (err)
		{
			printk(KERN_INFO "fat_find_first_free_clusters fat_ent_read_block2 failed\n");
			start = 0;
			goto out1;
		}

		do
		{
			if (ops->ent_get(&fatent) == FAT_ENT_FREE)
			{
				free++;

				if (start == 0)
				{
					start = fatent.entry;
				}

				clusters++;

				if (clusters >= clusterSum)
				{
					//printk(KERN_INFO "fat_find_free_clusters start = %ld\n", start);

					start--;
					if (start < FAT_START_ENT)
					{
						start = sbi->max_cluster - 1;
					}

					sbi->prev_free = start;
					sb->s_dirt = 1;

					*contiguous = 1;

					goto out1;
				}
			}
			else
			{
				start = 0;
				clusters = 0;
			}
			if (fatent.entry >= sbi->max_cluster)
			{
				start = 0;
				break;
			}
		} while (fat_ent_next2(sbi, &fatent));
	}
out1:
	//sbi->free_clusters = free;
	//sbi->free_clus_valid = 1;
	//sb->s_dirt = 1;
	fatent_brelse(&fatent);
out2:
	unlock_fat2(sbi);

	return start;
}

int fat_set_prev_free_cluster(struct super_block *sb, u_int32_t newLcn)
{
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	int ret = 0;

	lock_fat2(sbi);

	if (sbi->free_clusters != -1 && sbi->free_clus_valid && sbi->free_clusters < 1)
	{
		printk(KERN_INFO "fat_set_prev_free_cluster failed, free_clusters:%d, free_clus_valid:%d\n",
			sbi->free_clusters, sbi->free_clus_valid);
		goto out;
	}

	//printk(KERN_INFO "!!!fat_set_prev_free_cluster newLcn = %d\n", newLcn);

	newLcn--;
	if (newLcn < FAT_START_ENT)
	{
		newLcn = sbi->max_cluster - 1;
	}

	sbi->prev_free = newLcn;
	sb->s_dirt = 1;
	ret = 1;

out:
	unlock_fat2(sbi);

	return ret;
}

int fat_find_first_free_clusters(struct super_block *sb,
	u_int32_t MinimumLcn, u_int32_t MinimumSize,  u_int32_t *BeginLcn, u_int32_t *EndLcn)
{
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct fatent_operations *ops = sbi->fatent_ops;
	struct fat_entry fatent;
	unsigned long reada_blocks, reada_mask, cur_block;
	int err = 0;
	u_int32_t start = 0;

	*BeginLcn = 0;
	*EndLcn = 0;

	if (MinimumLcn >= sbi->max_cluster)
	{
		printk(KERN_INFO "fat_find_first_free_clusters minimum lcn is out of side\n");
		return 0;
	}

	lock_fat2(sbi);

	if (sbi->free_clusters != -1 && sbi->free_clus_valid && sbi->free_clusters < MinimumSize)
	{
		printk(KERN_INFO "fat_find_first_free_clusters failed, free_clusters:%d, free_clus_valid:%d\n",
			sbi->free_clusters, sbi->free_clus_valid);
		goto out2;
	}

	reada_blocks = FAT_READA_SIZE >> sb->s_blocksize_bits;
	reada_mask = reada_blocks - 1;
	cur_block = 0;

	fatent_init(&fatent);
	fatent_set_entry(&fatent, MinimumLcn);

	while (fatent.entry < sbi->max_cluster)
	{
		/* readahead of fat blocks */
		/*if ((cur_block & reada_mask) == 0)
		{
			unsigned long rest = sbi->fat_length - cur_block;
			fat_ent_reada2(sb, &fatent, min(reada_blocks, rest));
		}
		cur_block++;*/

		fatent_set_entry(&fatent, fatent.entry);

		err = fat_ent_read_block2(sb, &fatent);
		if (err)
		{
			printk(KERN_INFO "fat_find_first_free_clusters fat_ent_read_block2 failed\n");
			goto out1;
		}

		do
		{
			if (ops->ent_get(&fatent) == FAT_ENT_FREE)
			{
				if (start == 0)
				{
					start = fatent.entry;
				}

				if (fatent.entry >= sbi->max_cluster)
				{
					if ((fatent.entry - start) >= MinimumSize)
					{
						//printk(KERN_INFO "fat_find_first_free_clusters OK\n");
						*BeginLcn = start;
						*EndLcn = fatent.entry;
						goto out1;
					}
				}
			}
			else
			{
				if (0 != start)
				{
					if ((fatent.entry - start) >= MinimumSize)
					{
						*BeginLcn = start;
						*EndLcn = fatent.entry;
						goto out1;
					}
				}
				start = 0;
			}
		} while (fat_ent_next2(sbi, &fatent));

		if (fatent.entry >= sbi->max_cluster)
		{
			if (0 != start)
			{
				if ((fatent.entry - start) >= MinimumSize)
				{
					*BeginLcn = start;
					*EndLcn = fatent.entry;
				}
			}
			//printk(KERN_INFO "fat_find_first_free_clusters end...\n");
		}
	}
out1:
	//printk(KERN_INFO "fat_find_first_free_clusters out1:%d, val:%d, *BeginLcn:%d, sbi->fat_length:%ld, free_clusters:%d\n",
	//	fatent.entry, ops->ent_get(&fatent), *BeginLcn, sbi->fat_length, sbi->free_clusters);
	fatent_brelse(&fatent);
out2:
	unlock_fat2(sbi);

	return *BeginLcn;
}

int isNodeContinuous(struct inode *inode)
{
	struct fat_entry fatent;
	int dclus = 0;
	int nr = 0;
	struct msdos_inode_info * msinode= NULL;

	struct super_block *sb = NULL;
	struct msdos_sb_info *sbi = NULL;
	struct msdos_dir_entry *uninitialized_var(de);

	/*if (NULL != file && NULL != file->f_mapping)
	{
		inode = file->f_mapping->host;
	}
	else
	{
		printk(KERN_INFO "file or file->f_mapping is null, file:%x\n", (unsigned int)file);
		return 0;
	}*/

	if (NULL != inode)
	{
		msinode= MSDOS_I(inode);
		sb = inode->i_sb;
	}
	else
	{
		printk(KERN_INFO "inode is null\n");
		return 0;
	}
	//printk(KERN_INFO "3, inode:%x\n", (unsigned int)inode);

	if (NULL == msinode)
	{
		printk(KERN_INFO "msinode is null\n");
		return 0;
	}

	if (NULL != sb)
	{
		sbi = MSDOS_SB(sb);
	}

	if (NULL == sbi)
	{
		printk(KERN_INFO "sbi is null\n");
		return 0;
	}

	if (msinode->i_start < FAT_START_ENT || msinode->i_start > sbi->max_cluster)
	{
		printk(KERN_INFO "!!!start cluster is not correct\n");
		return 1;
	}

	lock_fat2(sbi);

	dclus = msinode->i_start;
	fatent_init(&fatent);

	while (dclus < FAT_ENT_EOF)
	{
		//printk(KERN_INFO "%d ", dclus);
		nr = fat_ent_read(inode, &fatent, dclus);
		if (FAT_ENT_EOF != nr && nr != (dclus + 1))
		{
			fatent_brelse(&fatent);
			unlock_fat2(sbi);
			printk(KERN_INFO "file is not contiguous\n");
			return 0;
		}

		dclus = nr;
	}

	fatent_brelse(&fatent);
	unlock_fat2(sbi);

	return 1;
}

int fat_relocation(struct super_block *sb, loff_t len)
{
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	unsigned long clusterNums = 0, start = 0;
	int contiguous = 0;

	clusterNums = (len + sbi->cluster_size - 1) >> sbi->cluster_bits;
	//printk(KERN_INFO "fat_relocation clusterNums:%ld\n", clusterNums);
	start = fat_find_free_clusters(sb, clusterNums, &contiguous);
	//printk(KERN_INFO "start:%ld, prevfree:%d\n", start, sbi->prev_free);

	return contiguous;
}

static int fat_cont_expand(struct inode *inode, loff_t size)
{
	struct address_space *mapping = inode->i_mapping;
	loff_t start = inode->i_size, count = size - inode->i_size;
	int err;

	err = generic_cont_expand_simple(inode, size);
	if (err)
		goto out;

	inode->i_ctime = inode->i_mtime = CURRENT_TIME_SEC;
	mark_inode_dirty(inode);
	if (IS_SYNC(inode)) {
		int err2;

		/*
		 * Opencode syncing since we don't have a file open to use
		 * standard fsync path.
		 */
		err = filemap_fdatawrite_range(mapping, start,
					       start + count - 1);
		err2 = sync_mapping_buffers(mapping);
		if (!err)
			err = err2;
		err2 = write_inode_now(inode, 1);
		if (!err)
			err = err2;
		if (!err) {
			err =  filemap_fdatawait_range(mapping, start,
						       start + count - 1);
		}
	}
out:
	return err;
}

/*
 * preallocate space for a file. This implements fat's fallocate file
 * operation, which gets called from sys_fallocate system call. User
 * space requests len bytes at offset.If FALLOC_FL_KEEP_SIZE is set
 * we just allocate clusters without zeroing them out.Otherwise we
 * allocate and zero out clusters via an expanding truncate.
 */
static long fat_fallocate(struct inode *inode, int mode, loff_t offset, loff_t len, u_int32_t newLcn)
{
	int err = 0;
	int contiguous = 0;
	//int cluster;
	//int fclus, dclus, nr_cluster;
	//int free_bytes, nr_bytes;
	struct super_block *sb = inode->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	//struct msdos_inode_info * msinode= MSDOS_I(inode);
	unsigned long clusterNums = 0, start = 0;


	/* No support for hole punch or other fallocate flags. */
	if (mode & ~FALLOC_FL_KEEP_SIZE)
		return -EOPNOTSUPP;


	//if (inode->i_size > 0)
	//{
	//	err = pfat_get_cluster(inode, FAT_ENT_EOF,
	//					&fclus, &dclus);
	//}

	//printk(KERN_INFO "stage 0, system type:%s, cluster bit size:%d, size:%d, fats:%d, inode.size:%d\n",
	//	sb->s_type->name, sbi->cluster_bits, sbi->cluster_size, sbi->fats, (int)(inode->i_size));
	//printk(KERN_INFO "fclus:%d, dclus:%d\n", fclus, dclus);
	//printk(KERN_INFO "msinode->i_start:%d, msinode->i_logstart:%d, mmu_private:%d, prevfree:%d\n",
	//	msinode->i_start, msinode->i_logstart, (int)msinode->mmu_private, sbi->prev_free);

	if (0 == newLcn)
	{
		clusterNums = (len + sbi->cluster_size - 1) >> sbi->cluster_bits;
		//printk(KERN_INFO "clusterNums:%ld\n", clusterNums);
		start = fat_find_free_clusters(sb, clusterNums, &contiguous);
		//printk(KERN_INFO "start:%ld, prevfree:%d, contiguous:%d\n", start, sbi->prev_free, contiguous);
		if (!contiguous)
		{
			return -EINVAL;
		}
	}
	else
	{
		if (!fat_set_prev_free_cluster(sb, newLcn))
		{
			printk(KERN_INFO "fat_set_prev_free_cluster failed\n");
			return -EINVAL;
		}
	}

	if ((offset + len) <= MSDOS_I(inode)->mmu_private)
	{
		fat_msg(sb, KERN_ERR,
				"fat_fallocate():Blocks already allocated");
		return -EINVAL;
	}

	//return err;

	if ((mode & FALLOC_FL_KEEP_SIZE))
	{
		// First compute the number of clusters to be allocated
		/*if (inode->i_size > 0)
		{
			printk(KERN_INFO "stage 1\n");
			err = pfat_get_cluster(inode, FAT_ENT_EOF,
					&fclus, &dclus);
			if (err < 0)
			{
				fat_msg(sb, KERN_ERR,
						"fat_fallocate():fat_get_cluster() error");
				return err;
			}
			free_bytes = ((fclus+1) << sbi->cluster_bits) -
				(inode->i_size);
			nr_bytes = (offset + len - inode->i_size) - free_bytes;

			printk(KERN_INFO "stage 1.5 free_bytes:%d, nr_bytes:%d\n", free_bytes, nr_bytes);
		}
		else
		{
			nr_bytes = (offset + len - inode->i_size);
		}

		printk(KERN_INFO "stage 2\n");

		nr_cluster = (nr_bytes + (sbi->cluster_size - 1)) >>
			sbi->cluster_bits;
		mutex_lock(&inode->i_mutex);

		printk(KERN_INFO "stage 3, nr_bytes:%d, nr_cluster:%d\n", nr_bytes, nr_cluster);

		// Start the allocation.We are not zeroing out the clusters
		while (nr_cluster-- > 0)
		{
			//printk(KERN_INFO "stage 4, nr_cluster:%d\n", nr_cluster);

			err = pfat_alloc_clusters(inode, &cluster, 1);
			//printk(KERN_INFO "stage 4, alloc cluster:%d\n", cluster);

			if (err)
			{
				fat_msg(sb, KERN_ERR,
						"fat_fallocate():fat_alloc_clusters() error");
				printk(KERN_INFO "stage 4.1\n");
				goto error;
			}
			err = pfat_chain_add(inode, cluster, 1);
			if (err)
			{
				printk(KERN_INFO "stage 4.2\n");
				fat_free_clusters(inode, cluster);
				goto error;
			}
		}

		printk(KERN_INFO "stage 5\n");

		// update mmu_private to allow writing to allocated clusters
		err = pfat_get_cluster(inode, FAT_ENT_EOF, &fclus, &dclus);
		if (err < 0)
		{
			printk(KERN_INFO "stage 6\n");

			fat_msg(sb, KERN_ERR,
					"fat_fallocate():fat_get_cluster() error");
			goto error;
		}
		MSDOS_I(inode)->mmu_private = (fclus + 1) << sbi->cluster_bits;

		printk(KERN_INFO "stage 7\n");

		err = inode_newsize_ok(inode, len);
		printk(KERN_INFO "new size, err:%d\n", err);

		err = fat_sync_inode(inode);
		printk(KERN_INFO "fat_sync_inode, err:%d\n", err);*/
	}
	else
	{
		//printk(KERN_INFO "stage 7.5\n");

		mutex_lock(&inode->i_mutex);
		/* This is just an expanding truncate */
		err = fat_cont_expand(inode, (offset + len));
		//printk(KERN_INFO "stage 7.6\n");
		if (err)
		{
			//printk(KERN_INFO "stage 7.8\n");
			fat_msg(sb, KERN_ERR, "fat_fallocate():fat_cont_expand() error");
		}
		/*else
		{
			struct fat_entry fatent;
			int next = 0;

			fatent_init(&fatent);
			next = pfat_ent_read(inode, &fatent, msinode->i_start);
			err = fat_free_clusters(inode, next);
			printk(KERN_INFO "fat_free_clusters, err:%d, msinode->i_start:%d, next:%d\n", err, msinode->i_start, next);
			//fat_fallocate(file, FALLOC_FL_KEEP_SIZE, 0, len);
		}*/
	}

	//printk(KERN_INFO "stage 8\n");

//error:

	//printk(KERN_INFO "stage 9\n");

	mutex_unlock(&inode->i_mutex);
	return err;
}

static long fat_fallocate_old(struct inode *inode, int mode, loff_t offset, loff_t len)
{
	int err = 0;
	struct super_block *sb = inode->i_sb;
	//struct msdos_sb_info *sbi = MSDOS_SB(sb);
	//struct msdos_inode_info * msinode= MSDOS_I(inode);


	/* No support for hole punch or other fallocate flags. */
	if (mode & ~FALLOC_FL_KEEP_SIZE)
		return -EOPNOTSUPP;

	if ((offset + len) <= MSDOS_I(inode)->mmu_private)
	{
		fat_msg(sb, KERN_ERR,
				"fat_fallocate():Blocks already allocated");
		return -EINVAL;
	}

	if ((mode & FALLOC_FL_KEEP_SIZE))
	{
		// not implemented...
	}
	else
	{
		//printk(KERN_INFO "stage 7.5\n");

		mutex_lock(&inode->i_mutex);
		/* This is just an expanding truncate */
		err = fat_cont_expand(inode, (offset + len));
		//printk(KERN_INFO "stage 7.6\n");
		if (err)
		{
			//printk(KERN_INFO "stage 7.8\n");
			fat_msg(sb, KERN_ERR, "fat_fallocate():fat_cont_expand() error");
		}
	}

	mutex_unlock(&inode->i_mutex);
	return err;
}

static int fat_fallocate_dir(struct super_block *sb, loff_t len, u_int32_t newLcn)
{
	if (0 == newLcn)
	{
		if (!fat_relocation(sb, len))
		{
			printk(KERN_INFO "fat_relocation failed\n");
			return 0;
		}
	}
	else
	{
		if (!fat_set_prev_free_cluster(sb, newLcn))
		{
			printk(KERN_INFO "fat_set_prev_free_cluster failed\n");
			return 0;
		}
	}
	return 1;
}

int fat_get_boot_info(struct inode *inode, struct bootInfo *boot)
{
	struct super_block *sb = inode->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);

	if (!boot)
	{
		printk(KERN_INFO "fat_get_boot_info failed\n");
		return 0;
	}

	memset(boot, 0, sizeof(struct bootInfo));

	boot->ClusterSize = sbi->cluster_size;
	boot->SecPerClust = sbi->sec_per_clus;
	boot->NumClusters = sbi->max_cluster;
	boot->BlockSize = sb->s_blocksize;

	//printk(KERN_INFO "fat_get_boot_info ClusterSize:%d\n", boot->ClusterSize);
	//printk(KERN_INFO "fat_get_boot_info dir_start:%ld, data_start:%ld, fat_start:%d\n", sbi->dir_start, sbi->data_start, sbi->fat_start);

	return 1;
}

int fat_get_file_info(struct inode *inode, struct fileInfo *info)
{
	struct super_block *sb;
	struct msdos_sb_info *sbi;
	struct msdos_inode_info * msinode;

	if (!info || !inode)
	{
		printk(KERN_INFO "fat_get_file_info failed\n");
		return 0;
	}

    sb = inode->i_sb;
    sbi = MSDOS_SB(sb);
    msinode = MSDOS_I(inode);

	info->lcn = msinode->i_start;
	info->realClusters = (inode->i_size + sbi->cluster_size - 1) >> sbi->cluster_bits;
	info->uncompressedClusters = 0;

	//printk(KERN_INFO "!!!fat_get_file_info  inode->i_size:%lld, lcn:%d\n", inode->i_size, info->lcn);

	return 1;
}

long get_files_info(char * filesystem_type)
{
	struct fs_struct *fs ;
	struct vfsmount *mnt ;
	struct super_block *mnt_sb ;
	struct file_system_type *s_type;
	//int n = 0;

	spin_lock(&current->fs->lock);
	fs = current->fs;
	mnt = fs->pwd.mnt;
	mnt_sb = mnt-> mnt_sb ;
	s_type = mnt_sb -> s_type;
	//printk("PWD Filesystem Type is : %s/n",s_type->name);
	//n = copy_to_user(filesystem_type,s_type->name,strlen(s_type->name));
	strcpy(filesystem_type,s_type->name);
	//printk("PWD= %ld/n",mnt_sb->s_blocksize);
	spin_unlock(&current->fs->lock);

	return 0;
}
int fat_fadvise(struct inode *inode, loff_t offset, loff_t len, int advice)
{
	//struct file *file = fget(fd);
	struct address_space *mapping= inode->i_mapping;
	struct backing_dev_info *bdi;
	loff_t endbyte;			/* inclusive */
	pgoff_t start_index;
	pgoff_t end_index;
	//unsigned long nrpages;
	int ret = 0;


	if (S_ISFIFO(inode->i_mode))
	{
		ret = -ESPIPE;
		goto out;
	}

	//mapping = file->f_mapping;
	if (!mapping || len < 0)
	{
		ret = -EINVAL;
		goto out;
	}

	/* Careful about overflows. Len == 0 means "as much as possible" */
	endbyte = offset + len;
	if (!len || endbyte < len)
		endbyte = -1;
	else
		endbyte--;		/* inclusive */

	bdi = mapping->backing_dev_info;

	switch (advice)
	{
	case POSIX_FADV_DONTNEED:
		//if (!bdi_write_congested(mapping->backing_dev_info))
		//	__filemap_fdatawrite_range(mapping, offset, endbyte, WB_SYNC_NONE);

		/* First and last FULL page! */
		start_index = (offset+(PAGE_CACHE_SIZE-1)) >> PAGE_CACHE_SHIFT;
		end_index = (endbyte >> PAGE_CACHE_SHIFT);

		if (end_index >= start_index)
			invalidate_mapping_pages(mapping, start_index,
						end_index);
		break;
	default:
		ret = -EINVAL;
	}
out:
	return ret;
}

int fat_set_file_first_cluster(struct inode *inode, u_int32_t newLcn)
{
	struct msdos_inode_info * msInode= MSDOS_I(inode);
	int ret = 0;

	msInode->i_start = newLcn;
	msInode->i_logstart = newLcn;

	/*
	* Since generic_write_sync() synchronizes regular files later,
	* we sync here only directories.
	*/
	if (S_ISDIR(inode->i_mode) && IS_DIRSYNC(inode))
	{
		ret = fat_sync_inode(inode);
		if (ret)
			return ret;
	}
	else
	{
		mark_inode_dirty(inode);
	}

	//printk("set first ret:%d\n", ret);

	return ret;
}


int fat_get_cluster_data(struct inode *inode, struct getClusterDataInfo *info)
{
	struct msdos_inode_info * msinode= MSDOS_I(inode);
	struct super_block *sb = inode->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct msdos_dir_entry *uninitialized_var(de);

	struct fat_entry fatent;
	struct buffer_head * bh=NULL;
	int dclus = 0, ret = 0;
	int i = 0, count = 0;
	unsigned char *buf = info->data;
	unsigned long sectorAddress = 0;


	if (msinode->i_start < FAT_START_ENT || msinode->i_start > sbi->max_cluster)
	{
		printk(KERN_INFO "!!!getClusterData:start cluster is not correct\n");
		return 1;
	}

	lock_fat2(sbi);

	dclus = msinode->i_start;
	fatent_init(&fatent);


	//boot->ClusterSize = sbi->cluster_size;
	//boot->SecPerClust = sbi->sec_per_clus;
	//boot->NumClusters = sbi->max_cluster;



	while (dclus < FAT_ENT_EOF)
	{
		sectorAddress = sbi->data_start + sbi->sec_per_clus * (dclus - 2);
		for (i = 0; i < sbi->sec_per_clus; i++)
		{
			bh = sb_bread(sb, sectorAddress);
			if (bh == NULL)
			{
				printk(KERN_ERR "!!!getClusterData: Directory bread failed\n");
				goto out;
			}

			if (copy_to_user(buf, bh->b_data, 512) != 0)
			{
				printk(KERN_ERR "!!!getClusterData: copy to user failed\n");
				goto out;
			}
			buf += 512;
			sectorAddress++;
		}
		count++;

		dclus = fat_ent_read(inode, &fatent, dclus);
		if (FAT_ENT_EOF == dclus && count != info->lcnNums)
		{
			printk(KERN_INFO "!!!getClusterData: cluster nums not correct\n");
			goto out;
		}
	}
	ret = 1;
out:
	fatent_brelse(&fatent);
	unlock_fat2(sbi);

	return ret;
}

int fat_get_clusters(struct inode *inode, struct getClustersInfo *info)
{
	struct fat_entry fatent;
	int dclus = 0;
	int index = 0;
	struct msdos_inode_info * msinode= MSDOS_I(inode);

	struct super_block *sb = inode->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct msdos_dir_entry *uninitialized_var(de);

	//printk(KERN_INFO "!!!fat_get_clusters start\n");

	if (msinode->i_start < FAT_START_ENT || msinode->i_start > sbi->max_cluster)
	{
		printk(KERN_INFO "!!!start cluster is not correct\n");
		return 0;
	}

	lock_fat2(sbi);

	dclus = msinode->i_start;
	fatent_init(&fatent);

	while (dclus < FAT_ENT_EOF)
	{
		//info->lcnList[index++] = dclus;
		if (copy_to_user(info->lcnList + index++, &dclus, sizeof(u_int32_t)) != 0)
		{
			printk(KERN_ERR "!!!getClusters: copy to user failed\n");
			goto out;
		}

		dclus = fat_ent_read(inode, &fatent, dclus);
		//printk(KERN_ERR "!!!getClusters entry:%d, nr_bhs:%d, bhs[0]:%x\n", fatent.entry, fatent.nr_bhs, (unsigned int)fatent.bhs[0]);
		// need to determine if the dclus is valid
	}

out:
	fatent_brelse(&fatent);
	unlock_fat2(sbi);

	return (index == info->lcnNum ? 1 : 0);
}

/**
 * double_down_write_data_sem - Acquire two inodes' write lock of i_data_sem
 *
 * @orig_inode:		original inode structure
 * @donor_inode:	donor inode structure
 * Acquire write lock of i_data_sem of the two inodes (orig and donor) by
 * i_ino order.
 */
void double_down_write_data_sem(struct inode *orig_inode, struct inode *donor_inode)
{
	struct inode *first = orig_inode, *second = donor_inode;

	/*
	 * Use the inode number to provide the stable locking order instead
	 * of its address, because the C language doesn't guarantee you can
	 * compare pointers that don't come from the same array.
	 */
	if (donor_inode->i_ino < orig_inode->i_ino) {
		first = donor_inode;
		second = orig_inode;
	}

	down_write(&MSDOS_I(first)->truncate_lock);
	down_write_nested(&MSDOS_I(second)->truncate_lock, SINGLE_DEPTH_NESTING);
}

/**
 * double_up_write_data_sem - Release two inodes' write lock of i_data_sem
 *
 * @orig_inode:		original inode structure to be released its lock first
 * @donor_inode:	donor inode structure to be released its lock second
 * Release write lock of i_data_sem of two inodes (orig and donor).
 */
void double_up_write_data_sem(struct inode *orig_inode, struct inode *donor_inode)
{
	up_write(&MSDOS_I(orig_inode)->truncate_lock);
	up_write(&MSDOS_I(donor_inode)->truncate_lock);
}

/**
 * mext_check_null_inode - NULL check for two inodes
 *
 * If inode1 or inode2 is NULL, return -EIO. Otherwise, return 0.
 */
static int
mext_check_null_inode(struct inode *inode1, struct inode *inode2,
		      const char *function, unsigned int line)
{
	int ret = 0;

	if (inode1 == NULL) {

		ret = -EIO;
	} else if (inode2 == NULL) {

		ret = -EIO;
	}
	return ret;
}

/**
 * mext_inode_double_lock - Lock i_mutex on both @inode1 and @inode2
 *
 * @inode1:	the inode structure
 * @inode2:	the inode structure
 *
 * Lock two inodes' i_mutex by i_ino order.
 * If inode1 or inode2 is NULL, return -EIO. Otherwise, return 0.
 */
static int
mext_inode_double_lock(struct inode *inode1, struct inode *inode2)
{
	int ret = 0;

	BUG_ON(inode1 == NULL && inode2 == NULL);

	ret = mext_check_null_inode(inode1, inode2, __func__, __LINE__);
	if (ret < 0)
		goto out;

	if (inode1 == inode2) {
		mutex_lock(&inode1->i_mutex);
		goto out;
	}

	if (inode1->i_ino < inode2->i_ino) {
		mutex_lock_nested(&inode1->i_mutex, I_MUTEX_PARENT);
		mutex_lock_nested(&inode2->i_mutex, I_MUTEX_CHILD);
	} else {
		mutex_lock_nested(&inode2->i_mutex, I_MUTEX_PARENT);
		mutex_lock_nested(&inode1->i_mutex, I_MUTEX_CHILD);
	}

out:
	return ret;
}

/**
 * mext_inode_double_unlock - Release i_mutex on both @inode1 and @inode2
 *
 * @inode1:     the inode that is released first
 * @inode2:     the inode that is released second
 *
 * If inode1 or inode2 is NULL, return -EIO. Otherwise, return 0.
 */

static int mext_inode_double_unlock(struct inode *inode1, struct inode *inode2)
{
	int ret = 0;

	BUG_ON(inode1 == NULL && inode2 == NULL);

	ret = mext_check_null_inode(inode1, inode2, __func__, __LINE__);
	if (ret < 0)
		goto out;

	if (inode1)
		mutex_unlock(&inode1->i_mutex);

	if (inode2 && inode2 != inode1)
		mutex_unlock(&inode2->i_mutex);

out:
	return ret;
}

int fat_mirror_bhs(struct super_block *sb, struct buffer_head **bhs,
			  int nr_bhs);

int fat_get_block(struct inode *inode, sector_t iblock,
			 struct buffer_head *bh_result, int create);

int fat_replace_one_cluster(struct inode *inode_orig, struct inode *inode_donor, struct moveCluster clst_orig, struct moveCluster clst_donor)
{
	struct fat_entry fatent;
	struct buffer_head *bhs[1];

	int dclus = 0;
	int err = 0;

	struct msdos_inode_info * msinode_orig= NULL;
	struct msdos_inode_info * msinode_donor= NULL;

	struct super_block *sb = inode_orig->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct msdos_dir_entry *uninitialized_var(de);
	struct fatent_operations *ops = sbi->fatent_ops;


	msinode_orig= MSDOS_I(inode_orig);
	msinode_donor= MSDOS_I(inode_donor);

	if (msinode_orig->i_start < FAT_START_ENT || msinode_orig->i_start > sbi->max_cluster)
	{
		printk(KERN_INFO "!!!start cluster is not correct\n");
		return 0;
	}

	lock_fat2(sbi);

	printk(KERN_INFO "!!!start fat_replace_one_cluster\n");

	// step 1: change the prev clusters
	if (msinode_orig->i_start == clst_orig.lcn)
	{
		printk(KERN_INFO "!!!fat_replace_one_cluster i_start\n");

		msinode_orig->i_start = clst_donor.lcn;
		msinode_orig->i_logstart = clst_donor.lcn;

		mark_inode_dirty(inode_orig);
	}
	else
	{
		printk(KERN_INFO "!!!fat_replace_one_cluster not i_start\n");

		dclus = clst_orig.lcnPrev;
		fatent_init(&fatent);
		dclus = fat_ent_read(inode_orig, &fatent, dclus);
		ops->ent_put(&fatent, clst_donor.lcn);
		sb->s_dirt = 1;
	}

	// step 2: swap the moved clusters
	dclus = clst_donor.lcn;
	fatent_init(&fatent);
	dclus = fat_ent_read(inode_orig, &fatent, dclus);
	ops->ent_put(&fatent, clst_orig.lcnNext);

	get_bh(fatent.bhs[0]);
	bhs[0] = fatent.bhs[0];

	sb->s_dirt = 1;



	mark_inode_dirty(inode_orig);

	unlock_fat2(sbi);

	fatent_brelse(&fatent);

	if (inode_needs_sync(inode_orig))
	{
		write_dirty_buffer(bhs[0], WRITE);
		wait_on_buffer(bhs[0]);
		if (!err && !buffer_uptodate(bhs[0]))
			err = -EIO;

		//err = fat_sync_bhs(bh, nr_bhs);
	}
	if (!err)
		err = fat_mirror_bhs(sb, bhs, 1);
	brelse(bhs[0]);

	return 1;
}

int fat_move_clusters(struct inode *inode_orig, struct moveClustersInfo *info)
{
	struct file *filp_orig = NULL;
	struct file *filp_donor = NULL;

	//struct inode *inode_orig = NULL;
	struct inode *inode_donor = NULL;

	struct fat_entry fatent;
	struct msdos_inode_info * msinode_orig= NULL;
	struct msdos_inode_info * msinode_donor= NULL;

	struct super_block *sb = inode_orig->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	struct msdos_dir_entry *uninitialized_var(de);
	struct fatent_operations *ops = sbi->fatent_ops;

	struct address_space *mapping = inode_orig->i_mapping;
	struct buffer_head *bh;
	struct page *page = NULL;
	const struct address_space_operations *a_ops = mapping->a_ops;

	__u32 orig_blk_offset = 0;

	int dclus = 0;
	int err = 0, i = 0;
	int err2 = 0;
	int ret1, ret2;
	int data_offset_in_page = 0;
	int block_len_in_page = PAGE_CACHE_SIZE >> inode_orig->i_blkbits;

	long long offs = 0;//(info->orig_start << inode_orig->i_blkbits) >> PAGE_CACHE_SHIFT;
	unsigned int sum_size = 0, finished_size = 0, data_size = 0, replaced_size = 0;
	unsigned int w_flags = 0;


	void *fsdata = NULL;



	bh = NULL;
	ops = ops;
	dclus = dclus;
	fatent.entry = 0;

	printk(KERN_INFO "!!!start fat_move_clusters, orig_start:%lld, donor_start:%lld, len:%lld, page size:%ld\n",
		info->orig_start, info->donor_start, info->len, PAGE_CACHE_SIZE);
	printk(KERN_INFO "!!!orig.lcn:%d, orig:prev:%d, orig.next:%d\n",
		info->lcnOrig.lcn, info->lcnOrig.lcnPrev, info->lcnOrig.lcnNext);
	printk(KERN_INFO "!!!donor.lcn:%d, donor:prev:%d, donor.next:%d\n",
		info->lcnDonor.lcn, info->lcnDonor.lcnPrev, info->lcnDonor.lcnNext);


	filp_orig = fget(info->orig_fd);
	if (!filp_orig) return 0;

	filp_donor = fget(info->donor_fd);
	if (!filp_donor) return 0;

	//inode_orig = filp_orig->f_dentry->d_inode;
	inode_donor = filp_donor->f_dentry->d_inode;

	msinode_orig= MSDOS_I(inode_orig);
	msinode_donor= MSDOS_I(inode_donor);

	if (msinode_orig->i_start < FAT_START_ENT || msinode_orig->i_start > sbi->max_cluster)
	{
		printk(KERN_INFO "!!! fat_move_clusters: start cluster is not correct\n");
		return 0;
	}




	err = mnt_want_write_file(filp_orig);
	if (err)
		goto out2;


	/* Protect orig and donor inodes against a truncate */
	ret1 = mext_inode_double_lock(inode_orig, inode_donor);
	if (ret1 < 0)
		return ret1;



	sum_size= info->len << inode_orig->i_blkbits;
	finished_size = 0;

	while (sum_size)// swap per page
	{
		printk(KERN_INFO "!!! fat_move_clusters    sum_size:%d\n", sum_size);

		if (segment_eq(get_fs(), KERNEL_DS))
			w_flags |= AOP_FLAG_UNINTERRUPTIBLE;

		orig_blk_offset = info->orig_start + (finished_size >> inode_orig->i_blkbits) + data_offset_in_page;

		offs = (long long)(info->orig_start << inode_orig->i_blkbits) + finished_size;
		//data_size = info->len << inode_orig->i_blkbits;
		data_size = PAGE_CACHE_SIZE;
		replaced_size = data_size;
		printk(KERN_INFO "\n!!! write begin  offs:%lld, data_size:%d, w_flags:%d, orig_blk_offset:%d\n", offs, data_size, w_flags, orig_blk_offset);
		err = a_ops->write_begin(filp_orig, mapping, offs, data_size, w_flags,
					 &page, &fsdata);
		printk(KERN_INFO "!!! err:%d\n", err);
		if (unlikely(err < 0))
			goto out;

		if (!PageUptodate(page))
		{
			printk(KERN_INFO "!!! up to date\n");
			mapping->a_ops->readpage(filp_orig, page);
			lock_page(page);
		}

		/*
		 * try_to_release_page() doesn't call releasepage in writeback mode.
		 * We should care about the order of writing to the same file
		 * by multiple move extent processes.
		 * It needs to call wait_on_page_writeback() to wait for the
		 * writeback of the page.
		 */
		wait_on_page_writeback(page);

		/* Release old bh and drop refs */
		try_to_release_page(page, 0);


		//replaced_size = mext_replace_branches(handle, orig_inode, donor_inode,
		//				orig_blk_offset, block_len_in_page, &err2);
		//if (err2 && replaced_size <= 0)
		//{
		//	goto out;
		//}
		fat_replace_one_cluster(inode_orig, inode_donor, info->lcnOrig, info->lcnDonor);
		fat_replace_one_cluster(inode_donor, inode_orig, info->lcnDonor, info->lcnOrig);

		if (!page_has_buffers(page))
		{
			printk(KERN_INFO "!!! create buffer\n");
			create_empty_buffers(page, 1 << inode_orig->i_blkbits, 0);
		}

		bh = page_buffers(page);
		printk(KERN_INFO "!!! data_offset_in_page:%d\n", data_offset_in_page);
		for (i = 0; i < data_offset_in_page; i++)
			bh = bh->b_this_page;

		for (i = 0; i < block_len_in_page; i++)
		{
			err = fat_get_block(inode_orig, (sector_t)(orig_blk_offset + i), bh, 0);
			printk(KERN_INFO "!!! fat get block err:%d, orig_blk_offset:%d\n", err, orig_blk_offset);
			if (err < 0)
				goto out;

			if (bh->b_this_page != NULL)
				bh = bh->b_this_page;
		}


		//printk(KERN_INFO "!!! write end\n");
		err = a_ops->write_end(filp_orig, mapping, offs, data_size, replaced_size,
				       page, fsdata);
		printk(KERN_INFO "!!! write end err:%d\n", err);
		page = NULL;


out:
		if (unlikely(page))
		{
			printk(KERN_INFO "!!! release page\n");

			if (PageLocked(page))
				unlock_page(page);
			page_cache_release(page);
			//ext4_journal_stop(handle);
		}

		sum_size -= replaced_size;
		finished_size += replaced_size;
	}
	//ext4_journal_stop(handle);

	if (err2)
		err = err2;

	printk(KERN_INFO "!!! out\n");

	ret2 = mext_inode_double_unlock(inode_orig, inode_donor);

	mnt_drop_write_file(filp_orig);

out2:

	printk(KERN_INFO "!!! out2\n");

	fput(filp_orig);
	fput(filp_donor);

	return 1;
}

int fat_ioctl_move_cluster(struct file *filp, u32 __user *user_arg)
{
	struct dataPkg pkg;
	//struct inode *inode = filp->f_dentry->d_inode;
	struct inode *inode = NULL;
	struct super_block *sb = filp->f_dentry->d_inode->i_sb;

	int ret = 0;

	//printk(KERN_INFO "!!!fat_ioctl_move_cluster --- Start\n");

	if (copy_from_user(&pkg, (struct dataPkg __user *)user_arg, sizeof(pkg)))
	{
		printk(KERN_INFO "!!!fat_ioctl_move_cluster copy pkg failed\n");
		return 0;
	}


	//printk(KERN_INFO "filename:%s, mode:%d, cmd:%d, inode:%x\n", p->fileName, p->mode, (int)(p->cmd), (unsigned int)inode);

	if (FALLOC_FLAG != pkg.checkFlag)
	{
		printk(KERN_INFO "!!!fallocate checkflag is not correct\n");
		return 0;
	}


	//printk(KERN_INFO "!!! i_blocks:%lld, s_blocksize:%ld\n", inode->i_blocks, sb->s_blocksize);
	//printk(KERN_INFO "!!!name:%s, i_count:%d\n", p->fileName, atomic_read(&inode->i_count));
	//printk(KERN_INFO "!!!i_blocks:%ld, i_blkbits:%d, i_size:%lld\n",
	//	(unsigned long)(inode->i_blocks), inode->i_blkbits, inode->i_size);

	switch (pkg.cmd)
	{
		case PKG_COMMAND_FALLOCATE_FILE:
		{
			struct fallocInfo info;
			ret = 0;
			if (NULL != pkg.data)
			{
				if (copy_from_user(&info, pkg.data, sizeof(struct fallocInfo))  == 0)
				{
					inode = ilookup(sb, pkg.inodeNumber);
					if (inode == 0) break;
                    if (0 != fat_fallocate(inode, info.mode, info.offset, info.len, info.newLcn))
					{
						ret = 0;
					}
					else
					{
						ret = isNodeContinuous(inode);
					}
					iput(inode);
				}
			}

			break;
		}

		case PKG_COMMAND_FALLOCATE_DIR:
		{
			struct fallocInfo info;
			ret = 0;
			if (NULL != pkg.data)
			{
				if (copy_from_user(&info, pkg.data, sizeof(struct fallocInfo))  == 0)
				{
					ret = fat_fallocate_dir(sb, info.len, info.newLcn);
				}
			}

			break;
		}

		case PKG_COMMAND_IFCONTIGUOUS:
			inode = ilookup(sb, pkg.inodeNumber);
			ret = isNodeContinuous(inode);
			iput(inode);

			break;

		case PKG_COMMAND_GET_BOOT_INFO:
		{
			struct bootInfo boot;

			inode = ilookup(sb, pkg.inodeNumber);
			if (fat_get_boot_info(inode, &boot))
			{
				ret = (copy_to_user(pkg.data, &boot, sizeof(struct bootInfo)) == 0 ? 1 : 0);
			}
			else
			{
				ret = 0;
			}
			iput(inode);

			//printk(KERN_INFO "!!!get boot info\n");
			break;
		}

		case PKG_COMMAND_GET_FILE_INFO:
		{
			struct fileInfo info;

			inode = ilookup(sb, pkg.inodeNumber);
			if (fat_get_file_info(inode, &info))
			{
				ret = (copy_to_user(pkg.data, &info, sizeof(struct fileInfo)) == 0 ? 1 : 0);
			}
			else
			{
				ret = 0;
			}
			iput(inode);

			//printk(KERN_INFO "!!!get file info\n");
			break;
		}

		case PKG_COMMAND_GET_FREE_CLUSTER_INFO:
		{
			struct freeClusterInfo  info;

			ret = 0;
			if (NULL != pkg.data)
			{
				if (copy_from_user(&info, pkg.data, sizeof(struct freeClusterInfo))  == 0)
				{
					//printk(KERN_INFO "lcn:%d, size:%d\n", info.MinimumLcn, info.MinimumSize);
					if (fat_find_first_free_clusters(sb, info.MinimumLcn, info.MinimumSize,  &info.BeginLcn, &info.EndLcn))
					{
						//printk(KERN_INFO "begin:%d, end:%d\n", info.BeginLcn, info.EndLcn);
						ret = (copy_to_user(pkg.data, &info, sizeof(struct freeClusterInfo)) == 0 ? 1 : 0);
					}
					else
					{
						printk(KERN_INFO "fat_find_first_free_clusters failed\n");
					}
				}
			}

			break;
		}

		case PKG_COMMAND_GET_FILE_REF_COUNT:
		{
			struct refInfo  info;
			struct file *fp = NULL;

			//if (S_ISDIR(inode->i_mode))
			//{
			//	ret = 0;
			//	goto out;
			//}

			fp = filp_open(pkg.fileName, O_RDWR, 0);
			if (NULL != fp)
			{
				info.f_count = 0;

				if (fp->f_dentry)
				{
					spin_lock(&fp->f_dentry->d_lock);
					info.f_count = fp->f_dentry->d_count;
					spin_unlock(&fp->f_dentry->d_lock);
				}
				//info.f_count = atomic_long_read(&fp->f_count);
				if (info.f_count > 0) info.f_count--;

				filp_close(fp, 0);

				ret = (copy_to_user(pkg.data, &info, sizeof(struct refInfo)) == 0 ? 1 : 0);
			}
			else
			{
				printk(KERN_INFO "!!!get file:%s ref count failed\n", pkg.fileName);
				ret = 0;
			}

			break;
		}

		case PKG_COMMAND_SET_NEW_HEAD:
		{
			struct setNewHeadInfo info;

			ret = 0;
			if (NULL != pkg.data && copy_from_user(&info, pkg.data, sizeof(struct setNewHeadInfo))  == 0)
			{
				inode = ilookup(sb, pkg.inodeNumber);
				ret = fat_set_file_first_cluster(inode, info.newLcn);
				iput(inode);
			}

			break;
		}

		case PKG_COMMAND_GET_CLUSTER_DATA:
		{
			struct getClusterDataInfo info;

			ret = 0;
			if (copy_from_user(&info, pkg.data, sizeof(struct getClusterDataInfo)) == 0)
			{
				inode = ilookup(sb, pkg.inodeNumber);
				ret = fat_get_cluster_data(inode, &info);
				iput(inode);
			}

			break;
		}

		case PKG_COMMAND_FADVISE:
		{
			inode = ilookup(sb, pkg.inodeNumber);
			ret = fat_fadvise(inode, 0, inode->i_size, POSIX_FADV_DONTNEED);
			iput(inode);

			//printk("fadvise  ret:%d\n", ret);
			break;
		}

		case PKG_COMMAND_FALLOCATE_FILE_OLD:
		{
			struct fallocInfo info;
			ret = 0;
			if (NULL != pkg.data)
			{
				if (copy_from_user(&info, pkg.data, sizeof(struct fallocInfo))  == 0)
				{
					inode = ilookup(sb, pkg.inodeNumber);
					ret = (0 != fat_fallocate_old(inode, info.mode, info.offset, info.len) ? 0 : 1);
					iput(inode);
				}
			}

			break;
		}

		case PKG_COMMAND_GET_CLUSTERS:
		{
			struct getClustersInfo info;
			ret = 0;
			if (NULL != pkg.data)
			{
				if (copy_from_user(&info, pkg.data, sizeof(struct getClustersInfo))  == 0)
				{
					inode = ilookup(sb, pkg.inodeNumber);
					ret = fat_get_clusters(inode, &info);
					iput(inode);
				}
			}

			break;
		}

		case PKG_COMMAND_MOVE_CLUSTERS:
		{
			struct moveClustersInfo info;
			ret = 0;
			if (NULL != pkg.data)
			{
				if (copy_from_user(&info, pkg.data, sizeof(struct moveClustersInfo))  == 0)
				{
					//ret = fat_move_clusters(inode, &info);
				}
			}

			break;
		}

		default:
			ret = 0;
			break;
	}


//out:
	//printk(KERN_INFO "!!!fat_ioctl_move_cluster --- End\n");

	return ret;
}



