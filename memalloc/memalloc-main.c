/* General headers */
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <linux/cdev.h>
#include <linux/freezer.h>
#include <linux/highmem.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched/mm.h>
#include <linux/skbuff.h>
#include <linux/vmalloc.h>

/* File IO-related headers */
#include <asm/uaccess.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>

/* Sleep and timer headers */
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/sched/types.h>

#include "../common.h"

#if !defined(CONFIG_X86_64)
#include <asm/tlbflush.h>
#endif

/* Simple licensing stuff */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("student");
MODULE_DESCRIPTION("Project 4, CSE 330 Fall 2025");
MODULE_VERSION("0.01");

/* Calls which start and stop the ioctl teardown */
bool memalloc_ioctl_init(void);
void memalloc_ioctl_teardown(void);

/* Used to create a device */
static dev_t dev = 0;
static struct class *vmod_class;
static struct cdev vmod_cdev;

/* Project 2 Solution Variable/Struct Declarations */
#define MAX_PAGES 4096
#define MAX_ALLOCATIONS 100
#define MAX_PROCESSES 10

struct alloc_info allocations[MAX_ALLOCATIONS];
int num_allocations = 0;
int remaining_allocations = 100;
int remaining_pages = 4096;

struct alloc_info alloc_req;
struct free_info free_req;
struct sharepage_info sharepage_req;
struct accesspage_info accesspage_req;

/* Set a shared page address */
unsigned long shared_paddr = 0x0;
bool shared_page_write = false;

/* Page table allocation helper functions defined in kmod_helper.c */
pud_t *memalloc_pud_alloc(p4d_t *p4d, unsigned long vaddr);
pmd_t *memalloc_pmd_alloc(pud_t *pud, unsigned long vaddr);
void memalloc_pte_alloc(pmd_t *pmd, unsigned long vaddr);

#if defined(CONFIG_X86_64)
#define PAGE_PERMS_RW PAGE_SHARED
#define PAGE_PERMS_R PAGE_READONLY
#else
#define PAGE_PERMS_RW                                                          \
  __pgprot(_PAGE_DEFAULT | PTE_USER | PTE_NG | PTE_PXN | PTE_UXN | PTE_WRITE)
#define PAGE_PERMS_R                                                           \
  __pgprot(_PAGE_DEFAULT | PTE_USER | PTE_NG | PTE_PXN | PTE_UXN | PTE_RDONLY)
#endif

static int kernel_module_allocate_single_page(unsigned long vaddr, bool write,
                                              unsigned long provided_paddr) {
  unsigned long paddr;

  pgd_t *pgd;
  p4d_t *p4d;
  pud_t *pud;
  pmd_t *pmd;
  pte_t *pte;

  printk("entered alocSinglePage with vaddr: %lu\n", vaddr);

  // code here
  //  You need to perform the full page walk:
  //  1. Get PGD, check if mapped.
  //  2. Get P4D, if not mapped call memalloc_pud_alloc().
  //  3. Get PUD, if not mapped call memalloc_pmd_alloc().
  //  4. Get PMD, if not mapped call memalloc_pte_alloc().
  //  5. Get PTE and check if it's already present. If so, return -1.

  pgd = pgd_offset(current->mm, vaddr);
  if (pgd_none(*pgd)) {
    printk(
        "Error: pgd should always be mapped (something is really wrong!).\n");
    goto not_in_memory;
  }
  printk("PGD is allocated. \n");

  /* Return pointer to the P4D. pgd is the pointer of PGD, address is the
     logical address in the virtual memory space.*/
  p4d = p4d_offset(pgd, vaddr);
  if (p4d_none(*p4d) || p4d_bad(*p4d)) {
    // printk("No P4D allocated; page must be unmapped.");
    // goto not_in_memory;
    memalloc_pud_alloc(p4d, vaddr);
    p4d = p4d_offset(pgd, vaddr);
  }
  printk("P4D is allocated. \n");

  /* Return pointer to the PUD. p4d is the pointer of P4D, address is the
     logical address in the virtual memory space.*/
  pud = pud_offset(p4d, vaddr);
  if (pud_none(*pud)) {
    // printk("No PUD allocated; page must be unmapped.");
    // goto not_in_memory;
    memalloc_pmd_alloc(pud, vaddr);
    pud = pud_offset(p4d, vaddr);
  }
  printk("PUD is allocated. \n");

  /* Return pointer to the PMD. pud is the pointer of PUD, address is the
     logical address in the virtual memory space.*/
  pmd = pmd_offset(pud, vaddr);
  if (pmd_none(*pmd)) {
    // printk("No PMD allocated; page must be unmapped.");
    // goto not_in_memory;
    memalloc_pte_alloc(pmd, vaddr);
    pmd = pmd_offset(pud, vaddr);
  }
  printk("PMD is allocated. \n");

  /* Return pointer to the PTE. pmd is the pointer of PMD, address is the
     logical address in the virtual memory space*/
  pte = pte_offset_kernel(pmd, vaddr);
  if (pte_none(*pte)) {
    printk("No PTE allocated; page must be unmapped.");
    // goto not_in_memory;
  }
  // printk("PTE is allocated. \n");

  if (pte_present(*pte)) {
    printk("Page is mapped.\n");
    return -1;
  } else {
    printk("Page is not mapped. \n"); // Get a free page for the process
    goto not_in_memory;
  }

  // gfp_t gfp = GFP_KERNEL_ACCOUNT;
  // void *virt_addr = (void*) get_zeroed_page(gfp);
  // if (!virt_addr) {
  //     printk(KERN_ERR "Failed to allocate memory using vmalloc\n");
  //     return -ENOMEM;
  // }

not_in_memory:
  if (provided_paddr == 0) {
    gfp_t gfp = GFP_KERNEL_ACCOUNT;
    void *virt_addr = (void *)get_zeroed_page(gfp);
    if (!virt_addr) {
      printk(KERN_ERR "Failed to allocate memory using vmalloc\n");
      return -ENOMEM;
    }

    // Get the physical address of the first page
    paddr = __pa(virt_addr);
  } else {
    /* For the page sharing case. */
    paddr = provided_paddr;
  }

  // Set the page table entry using the current memory manager (mm)
  if (write)
    set_pte_at(current->mm, vaddr, pte,
               pfn_pte((paddr >> PAGE_SHIFT), PAGE_PERMS_RW));
  else
    set_pte_at(current->mm, vaddr, pte,
               pfn_pte((paddr >> PAGE_SHIFT), PAGE_PERMS_R));

  printk("Allocating page ==> %lx\n", (unsigned long)vaddr);

  // Sanity check
  if (!pte_present(*pte)) {
    printk("Error: Page table entry is still not present.\n");
    return -1;
  }

  return 0;
}

/* Free routine */
static int kernel_module_free_single_page(unsigned long vaddr) {
  unsigned long paddr;

  pgd_t *pgd;
  p4d_t *p4d;
  pud_t *pud;
  pmd_t *pmd;
  pte_t *pte;

  // code here
  //  1. Follow the same page walk as in allocation.
  //  2. After each, check if the entry is mapped. If not, return -1 and print a
  //  failure message.
  //  3. At the PTE level, check if pte_present(*pte) — if not, print a message
  //  and return -1.
  //  4. If valid, clear the PTE.

  printk("entered freeSinglePage with %lu\n", vaddr);
  pgd = pgd_offset(current->mm, vaddr);
  if (pgd_none(*pgd)) {
    printk(
        "Error: pgd should always be mapped (something is really wrong!).\n");
    return -1;
  }
  // printk("PGD is allocated. \n");

  /* Return pointer to the P4D. pgd is the pointer of PGD, address is the
     logical address in the virtual memory space.*/
  p4d = p4d_offset(pgd, vaddr);
  if (p4d_none(*p4d) || p4d_bad(*p4d)) {
    // printk("No P4D allocated; page must be unmapped.");
    // goto not_in_memory;
    return -1;
  }
  // printk("P4D is allocated. \n");

  /* Return pointer to the PUD. p4d is the pointer of P4D, address is the
     logical address in the virtual memory space.*/
  pud = pud_offset(p4d, vaddr);
  if (pud_none(*pud)) {
    // printk("No PUD allocated; page must be unmapped.");
    // goto not_in_memory;
    return -1;
  }
  // printk("PUD is allocated. \n");

  /* Return pointer to the PMD. pud is the pointer of PUD, address is the
     logical address in the virtual memory space.*/
  pmd = pmd_offset(pud, vaddr);
  if (pmd_none(*pmd)) {
    // printk("No PMD allocated; page must be unmapped.");
    // goto not_in_memory;pud_of
    return -1;
  }

  // printk("PMD is allocated. \n");

  /* Return pointer to the PTE. pmd is the pointer of PMD, address is the
     logical address in the virtual memory space*/
  pte = pte_offset_kernel(pmd, vaddr);
  if (pte_none(*pte)) {
    // printk("No PTE allocated; page must be unmapped.");
    // goto not_in_memory;
    return -1;
  }
  // printk("PTE is allocated. \n");

  if (!pte_present(*pte)) {
    // printk("Page is mapped.");
    return -1;
  }

  pte_clear(current->mm, vaddr, pte);

  // Flush the TLB entry
#if defined(CONFIG_X86_64)
  asm volatile("invlpg (%0)" ::"r"(vaddr) : "memory");
#else
  local_flush_tlb_all();
#endif
  return 0;
}

static int set_shared_page(struct sharepage_info sharepage_req) {
  unsigned long vaddr = sharepage_req.vaddr;
  shared_page_write = sharepage_req.write;

  /* Walk the page tables to find the physical address */
  pgd_t *pgd;
  p4d_t *p4d;
  pud_t *pud;
  pmd_t *pmd;
  pte_t *pte;

  pgd = pgd_offset(current->mm, vaddr);
  if (pgd_none(*pgd))
    return -1;

  p4d = p4d_offset(pgd, vaddr);
  if (p4d_none(*p4d))
    return -1;

  pud = pud_offset(p4d, vaddr);
  if (pud_none(*pud))
    return -1;

  pmd = pmd_offset(pud, vaddr);
  if (pmd_none(*pmd))
    return -1;

  pte = pte_offset_kernel(pmd, vaddr);
  if (pte_none(*pte))
    return -1;

  /* If the pte is present, it is mapped. Thus, we should return with an error.
   */
  if (!pte_present(*pte)) {
    printk("Mapping is not present in memory\n");
    return -1;
  }

  /* Get the physical address from the page. */
  struct page *pg = pte_page(*pte);
  shared_paddr = page_to_phys(pg);
  printk("Physical address to be shared => %px\n", shared_paddr);
  return 0;
}

static int kernel_module_allocate_pages(struct alloc_info alloc_req) {
  unsigned long vaddr;
  int num_pages;
  vaddr = alloc_req.vaddr;
  bool write = alloc_req.write;
  num_pages = alloc_req.num_pages;

  /* Set maximum pages to maximum allowed allocations */
  if (remaining_allocations == 0) {
    printk("Exceeded allocation request limit.\n");
    return -3;
  }
  if (((int)remaining_pages - num_pages) < 0) {
    printk("Exceeded page request limit.\n");
    return -2;
  }

  /* Only allocate maximum allocation pages */
  int ret = 0;
  printk("entered allocPages with vaddr: %lu\n", vaddr);
  while (num_pages > 0) {
    // code here
    // Call kernel_module_allocate_single_page function
    // If return value is -1, log allocation failed and return -1
    // Otherwise, update vaddr and num_pages
    ret = kernel_module_allocate_single_page(vaddr, write, 0);
    if (ret == -1) {
      // fixme
      printk("alloc failed and returned -1, check single_page\n");
      return -1;
    }

    num_pages--;
    vaddr += PAGE_SIZE;
  }

  // Store allocation information in our local array.
  // Fill in the allocations[] array with vaddr and num_pages.
  // Then update num_allocations, remaining_allocations, and remaining_pages
  // accordingly.
  allocations[num_allocations].vaddr = alloc_req.vaddr;
  allocations[num_allocations].write = alloc_req.write;
  allocations[num_allocations].num_pages = alloc_req.num_pages;
  remaining_pages -= alloc_req.num_pages;
  num_allocations++;
  remaining_allocations--;

  printk(KERN_INFO "Allocated %d page(s) at virtual address %lx \n",
         alloc_req.num_pages, alloc_req.vaddr);
  printk("[allocations] current = %d, remaining = %d\n", num_allocations,
         remaining_allocations);
  printk("[pages] current = %d, remaining = %d\n", alloc_req.num_pages,
         remaining_pages);
  return 0;
}

static void kernel_module_free_pages(unsigned long vaddr) {
  int i;

  printk("Entered freePages with vaddr = %lu\n", vaddr);

  // Find the allocation entry
  for (i = 0; i < num_allocations; ++i) {
    // code here
    // This loop goes through each recorded allocation to find the one matching
    // the given virtual address. If a match is found, the corresponding pages
    // are freed and the allocation entry is cleared.
    // fixme

    if (allocations[i].vaddr == vaddr) {
      kernel_module_free_single_page(vaddr);
      printk("page freed\n");
      return;
    }
  }

  printk(KERN_WARNING "No allocation found for virtual address %lx\n", vaddr);
}

/* Init and Exit functions */
static int __init memalloc_module_init(void) {
  /* Setup IOCTL */
  printk("Hello from the memalloc module!\n");
  if (!memalloc_ioctl_init()) {
    return -1;
  }
  return 0;
}

static void __exit memalloc_module_exit(void) {
  /* Teardown IOCTL */
  printk("Goodbye from the memalloc module!\n");
  memalloc_ioctl_teardown();
}

/* An open function for host module. */
static int memalloc_open(struct inode *inode, struct file *file) { return 0; }

/* A release function for the host module. */
static int memalloc_release(struct inode *inode, struct file *file) {
  return 0;
}

/* IOCTL handler for vmod. */
static long memalloc_ioctl(struct file *f, unsigned int cmd,
                           unsigned long arg) {
  switch (cmd) {
  case ALLOCATE:
    // code here
    // refer to the SHAREPAGE and ACCESSPAGE cases to write the code here.
    if (copy_from_user((void *)&alloc_req, (void *)arg,
                       sizeof(struct alloc_info))) {
      printk("User didn't send right message.\n");
    }

    // fixme potential error?
    kernel_module_allocate_pages(alloc_req);

    break;

  case FREE:
    // code here
    // refer to the SHAREPAGE and ACCESSPAGE cases to write the code here.
    if (copy_from_user((void *)&free_req, (void *)arg,
                       sizeof(struct free_info))) {
      printk("User didn't send right message.\n");
    }

    // fixme
    kernel_module_free_pages(free_req.vaddr);
    break;

  case SHAREPAGE:
    /* Share a page */
    if (copy_from_user((void *)&sharepage_req, (void *)arg,
                       sizeof(struct sharepage_info))) {
      printk("User didn't send right message.\n");
    }

    /* debug message */
    printk("IOCTL: sharepage(%lx, %d, %d)\n", sharepage_req.vaddr,
           sharepage_req.client_pid, sharepage_req.write);

    set_shared_page(sharepage_req);
    break;

  case ACCESSPAGE:
    /* Access a shared page */

    /* copy data from user */
    if (copy_from_user((void *)&accesspage_req, (void *)arg,
                       sizeof(struct accesspage_info))) {
      printk("User didn't send right message.\n");
    }

    /* debug message */
    printk("IOCTL: accesspage(%lx)\n", accesspage_req.vaddr);

    kernel_module_allocate_single_page(accesspage_req.vaddr, shared_page_write,
                                       shared_paddr);

    break;
  }

  /* Required file ops. */
  static struct file_operations fops = {
      .owner = THIS_MODULE,
      .open = memalloc_open,
      .release = memalloc_release,
      .unlocked_ioctl = memalloc_ioctl,
  };

  /* Initialize the module for IOCTL commands */
  bool memalloc_ioctl_init(void) {

    /* Allocate a character device. */
    if (alloc_chrdev_region(&dev, 0, 1, "memalloc") < 0) {
      printk("error: couldn't allocate chardev region.\n");
      return false;
    }
    printk("success: allocated chardev region.\n");

    /* Initialize the chardev with my fops. */
    cdev_init(&vmod_cdev, &fops);

    if (cdev_add(&vmod_cdev, dev, 1) < 0) {
      printk("error: couldn't add memalloc.\n");
      goto cdevfailed;
    }
    printk("success: added cdev.\n");

    if ((vmod_class = class_create("memalloc_class")) == NULL) {
      printk("error: couldn't create class.\n");
      goto cdevfailed;
    }
    printk("success: created class.\n");

    if ((device_create(vmod_class, NULL, dev, NULL, "memalloc")) == NULL) {
      printk("error: couldn't create device.\n");
      goto classfailed;
    }
    printk("success: memalloc device driver inserted.\n");

    return true;

  classfailed:
    class_destroy(vmod_class);
  cdevfailed:
    unregister_chrdev_region(dev, 1);

    return false;
  }

  void memalloc_ioctl_teardown(void) {
    /* Destroy the classes too (IOCTL-specific). */
    if (vmod_class) {
      device_destroy(vmod_class, dev);
      class_destroy(vmod_class);
    }
    cdev_del(&vmod_cdev);
    unregister_chrdev_region(dev, 1);

    printk("Removed vmod device driver from host.\n");
  }

  module_init(memalloc_module_init);
  module_exit(memalloc_module_exit);
