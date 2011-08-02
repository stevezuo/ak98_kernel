 @void mmu_clean_flash_dcache
            .global mmu_clean_flash_dcache
	mmu_clean_flash_dcache:
             mrc p15,0,r15,c7,c10,3
             bne mmu_clean_flash_dcache
             mov pc,lr

 
   @void mmu_clean_invalidate_dcache
            .global mmu_clean_invalidate_dcache
	mmu_clean_invalidate_dcache:
             mrc p15,0,r15,c7,c14,3
             bne mmu_clean_invalidate_dcache
             mov pc,lr

