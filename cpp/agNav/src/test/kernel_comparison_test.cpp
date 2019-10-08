#include <gtest/gtest.h>
#include <TraversabilityOcTree.h>
#include <tuple>

using namespace octomap;

class KernelInitParameterizedTestFixture : public ::testing::TestWithParam<std::tuple<float, float, float, float>> {
 protected:
  TraversabilityOcTree* tree;
};

TEST_P(KernelInitParameterizedTestFixture, KernelSizeTest)
{
  std::tuple<float, float, float, float> p = GetParam();
  tree = new TraversabilityOcTree(std::get<0>(p),
				  std::get<1>(p),
				  std::get<2>(p),
				  std::get<3>(p));
  float correct_size = ( ceilf(std::get<1>(p) / std::get<0>(p)) *
			 ceilf(std::get<2>(p) / std::get<0>(p)) *
			 ceilf(std::get<3>(p) / std::get<0>(p) + 1) );
  ASSERT_EQ( correct_size, tree->getKernelSize() );
  delete tree;
}

INSTANTIATE_TEST_CASE_P(
			KernelSizeTests,
			KernelInitParameterizedTestFixture,
                        ::testing::Combine(::testing::Range(0.1f, 1.0f, 0.1),
                                           ::testing::Range(0.1f, 0.5f, 0.1),
					   ::testing::Range(0.1f, 0.5f, 0.1),
					   ::testing::Range(0.1f, 0.5f, 0.1)));

TEST(KernelInitTest, MaxClampingTest)
{
  float t_res = 0.5;
  point3d target_voxel(t_res / 2, t_res / 2, t_res / 2);
  TraversabilityOcTree* tree = new TraversabilityOcTree(t_res, t_res, t_res, t_res);
  tree->updateNode(tree->coordToKey(target_voxel), 6.0f);
  ASSERT_EQ( tree->getClampingThresMax(), (tree->search( tree->coordToKey(target_voxel)))->getOccupancy());
}

TEST(KernelInitTest, MinClampingTest)
{
  float t_res = 0.5;
  point3d target_voxel(t_res / 2, t_res / 2, t_res / 2);
  TraversabilityOcTree* tree = new TraversabilityOcTree(t_res, t_res, t_res, t_res);
  tree->updateNode(tree->coordToKey(target_voxel), -6.0f);
  ASSERT_EQ( tree->getClampingThresMin(), (tree->search( tree->coordToKey(target_voxel)))->getOccupancy());
}

TEST(KernelComparisonTest, EmptyTree)
{
  float t_res = 0.5;
  TraversabilityOcTree* tree = new TraversabilityOcTree(t_res, t_res, t_res, t_res);
  point3d target_voxel(t_res / 2, t_res / 2, t_res / 2);
  float trav_score = tree->calculateTraversability(target_voxel) / tree->getKernelSize();
  ASSERT_EQ( 0.5, trav_score ) << "Traversability score in unknown space is not 0.5";
}

TEST(KernelComparisonTest, SingleNodeMax)
{
  float t_res = 0.5;
  float k_dim = t_res;
  float start = t_res / 2;
  
  point3d occ_coord(start, start, start);
  point3d free_coord(start, start, start + t_res);

  TraversabilityOcTree* tree = new TraversabilityOcTree(t_res, k_dim, k_dim, k_dim);
  tree->updateNode(tree->coordToKey(occ_coord), 6.0f);
  float trav_score = tree->calculateTraversability(occ_coord) / tree->getKernelSize();
  
  ASSERT_EQ( (0.5 * (tree->getKernelSize() - 1)) / tree->getKernelSize() , trav_score ) << "Error: Single traversable node in kernel has incorrect value";
}

// class KernelComparisonTestFixture : public ::testing::TestWithParam<std::tuple<float, float, float, float>> {
//  protected:
//   TraversabilityOcTree* tree;
//   int iter_start[3] = {(int)floorf( bbx_start.x() / t_res ),
// 		       (int)floorf( bbx_start.y() / t_res ),
// 		       (int)floorf( bbx_start.z() / t_res )};
  
//   int iter_end[3] ={(int)floorf( bbx_end.x() / t_res ),
// 		    (int)floorf( bbx_end.y() / t_res ),
// 		    (int)floorf( bbx_end.z() / t_res )};

// };

TEST(TrueLeafBbxTest, SplitSingleTreeNode)
{
  float t_res = 0.5;
  float k_dim = 0.2;
  float start = t_res/2;
  float end = start + 10;
  point3d bbx_start(start, start, start);
  point3d bbx_end(end, end, end);
  TraversabilityOcTree* tree = new TraversabilityOcTree(t_res, k_dim, k_dim, k_dim);

  int iter_start[3] = {(int)floorf( bbx_start.x() / t_res ),
		       (int)floorf( bbx_start.y() / t_res ),
		       (int)floorf( bbx_start.z() / t_res )};
  
  int iter_end[3] ={(int)floorf( bbx_end.x() / t_res ),
		    (int)floorf( bbx_end.y() / t_res ),
		    (int)floorf( bbx_end.z() / t_res )};

  int count = 0;

  for (int i = iter_start[0]; i < iter_end[0]; i++)
    {
    for (int j = iter_start[1]; j < iter_end[1]; j++)
      {
      for (int k = iter_start[2]; k < iter_end[2]; k++)
	{
	  point3d cur_node( i * t_res, j * t_res, k * t_res);
	  tree->updateNode(tree->coordToKey(cur_node), -6.0f);
	}
      }
    }
  
  for (int i = iter_start[0]; i < iter_end[0]; i++)
    {
    for (int j = iter_start[1]; j < iter_end[1]; j++)
      {
	point3d cur_node( (i + 0.5) * t_res, (j + 0.5) * t_res, (iter_start[2] + 0.5) * t_res);
	tree->updateNode(tree->coordToKey(cur_node), 6.0f);
      }
    }

  for (TraversabilityOcTree::true_leaf_bbx_iterator iter = TraversabilityOcTree::true_leaf_bbx_iterator(tree,
  													bbx_start,
  													bbx_end);
       iter.isStackEmpty() != true;
       ++iter)
    {
      count++;
    }

  ASSERT_EQ( roundf( ((bbx_end.x() - bbx_start.x()) *
  		      (bbx_end.y() - bbx_start.y()) *
  		      (bbx_end.z() - bbx_start.z())) / pow(t_res,3) ),
  	     count) << "Bounding box iterator iterations did not match expected";
  
  point3d occ_voxel(start, start, start);
  point3d free_voxel(start, start, start + t_res);
  ASSERT_EQ( tree->getClampingThresMax(), (tree->search( tree->coordToKey(occ_voxel)))->getOccupancy());
  ASSERT_EQ( tree->getClampingThresMin(), (tree->search( tree->coordToKey(free_voxel)))->getOccupancy());

  point3d target_voxel(start, start, start);
  float trav_score = tree->calculateTraversability(target_voxel) / tree->getKernelSize();
  ASSERT_EQ( 0, trav_score ) << "Pure traversabile node score is not 0";
  
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
