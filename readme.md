思路：
1.每次运行前都清空rgb和depth文件夹的内容
2.保存rgb和depth图片到/rgb和/depth
	（要求：rgb对depth 1对1     文件命名要以YCB 19位数字命名）
3.绘制第一张rgb图片掩码到mask 并用labelme打标
4.用混元ai进行能量单元的3d建模 并导出obj文件 最后在blender中调整能量单元的实际距离相对比例
