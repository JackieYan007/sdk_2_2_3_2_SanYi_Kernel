static int32_t bstn_mempool_init(struct bstn *bstn)
{
	int32_t ret = 0;
	ret = of_reserved_mem_device_init(bstn->dev);
	if (ret < 0) {
		BSTN_DEV_ERR(bstn->dev, "of_reserved_mem_device_init fail, ret: %d\n", ret);
		return -ENODEV;
	}
	return bstn_init_cma_mempool(&bstn->pool, bstn->dev);
}

static int32_t bstn_probe(struct platform_device *pdev)
{
	//init bstn share mempool
    int32_t ret=0;
	ret = dma_set_coherent_mask(bstn->dev, DMA_BIT_MASK(36));	// TODO
	if (ret < 0) {
		BSTN_DEV_ERR(&pdev->dev, "dma_set_coherent_mask fail, ret %d\n", ret);
	}
	ret = bstn_mempool_init(bstn);
	if (ret < 0) {
		BSTN_DEV_ERR(&pdev->dev, "bstn_mempool_init fail, ret %d\n", ret);
		return ret;
	}

}


static int32_t bstn_remove(struct platform_device *pdev)
{
	struct bstn *bstn = platform_get_drvdata(pdev);

	BSTN_TRACE_PRINTK("%s","enter");

	bstn->status = BSTN_DESTROY;
	misc_deregister(&bstn->miscdev);
	bstn_destroy_cma_mempool(bstn->pool);
	bstn_release_fimware(bstn);

	BSTN_TRACE_PRINTK("%s", "exit");
	return 0;
}

static struct platform_driver bstn_driver = {
	.probe   = bstn_probe,
	.remove  = bstn_remove,
	.driver  = {
		.name = BSTN_DRIVER_NAME,
		.of_match_table = of_match_ptr(bstn_of_match),
		//.pm = &bstn_pm_ops,
	},
};

module_platform_driver(bstn_driver);

MODULE_AUTHOR("Xiaodong Liu");
MODULE_DESCRIPTION("BSTN: Linux device driver for Black Sesame Technologies Neural Network IP");
MODULE_LICENSE("GPL");

