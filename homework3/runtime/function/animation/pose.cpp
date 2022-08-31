#include "runtime/function/animation/pose.h"

using namespace Pilot;

AnimationPose::AnimationPose() { m_reorder = false; }

AnimationPose::AnimationPose(const AnimationClip& clip, float ratio, const AnimSkelMap& animSkelMap)
{
    m_bone_indexs = animSkelMap.convert;
    m_reorder     = true;
    extractFromClip(m_bone_poses, clip, ratio);
    m_weight.m_blend_weight.resize(m_bone_poses.size());
    for (auto& weight : m_weight.m_blend_weight)
    {
        weight = 1.f;
    }

}
AnimationPose::AnimationPose(const AnimationClip& clip, const BoneBlendWeight& weight, float ratio)
{
    m_weight  = weight;
    m_reorder = false;
    extractFromClip(m_bone_poses, clip, ratio);
}
AnimationPose::AnimationPose(const AnimationClip&   clip,
                             const BoneBlendWeight& weight,
                             float                  ratio,
                             const AnimSkelMap&     animSkelMap)
{
    m_weight      = weight;
    m_bone_indexs = animSkelMap.convert;
    m_reorder     = true;
    extractFromClip(m_bone_poses, clip, ratio);
}

void AnimationPose::extractFromClip(std::vector<Transform>& bones, const AnimationClip& clip, float ratio)
{
    bones.resize(clip.node_count);

    float exact_frame        = ratio * (clip.total_frame - 1);
    int   current_frame_low  = floor(exact_frame); 
    int   current_frame_high = ceil(exact_frame);
    float lerp_ratio         = exact_frame - current_frame_low;
    for (int i = 0; i < clip.node_count; i++)
    {
        const AnimationChannel& channel = clip.node_channels[i];
        bones[i].m_position = Vector3::lerp(
            channel.position_keys[current_frame_low], channel.position_keys[current_frame_high], lerp_ratio);
        bones[i].m_scale    = Vector3::lerp(
            channel.scaling_keys[current_frame_low], channel.scaling_keys[current_frame_high], lerp_ratio);
        bones[i].m_rotation = Quaternion::nLerp(
            lerp_ratio, channel.rotation_keys[current_frame_low], channel.rotation_keys[current_frame_high], true);
    }
}


void AnimationPose::blend(const AnimationPose& pose)
{
    for (int i = 0; i < m_bone_poses.size(); i++)
    {
        auto&       bone_trans_one = m_bone_poses[i];
        const auto& bone_trans_two = pose.m_bone_poses[i];

        //  m_weight 是一个 BoneBlendWeight
        // m_weight.m_blend_weight[i]应该是sum_weight用于累加做归一化的
        // desired_ratio应该是当前动画的播放进度（0-1的一个数值比例），存进了m_blend_ratio变量里，用于从参与混合的每个clip中提取该时间进度所对应的pose，然后再把这些pose做混合，得到该时间进度所对应的最终pose。作业说明里第二部分有提到

        float sum_weight = pose.m_weight.m_blend_weight[i] + m_weight.m_blend_weight[i];
        
        if (sum_weight != 0)
        {   
            // 归一化权重
            float cur_weight = pose.m_weight.m_blend_weight[i] / sum_weight;
            // 我感觉给不给都一样啊，反正下一个tick都会被重新赋值
            // 详细的解释：

            // m_weight.m_blend_weight[i]=sum_weight，是为了存储下一轮混合时A应该占多少权重。
            // AnimationComponent::blend的逻辑是先拿B跟A混合，存到A里；再拿C跟A混合，存到A里，最终得到三者的混合结果。
            // 假设pose A B C的权重分别是0.5 0.3 0.2
            // 第一次混合时B的归一化权重是：0.3/(0.3+0.5)（也就是cur_weight），A的归一化权重就是5/8。
            // 另外记录sum_weight为0.8，用于当作下次混合是A的权重（此时的A存储的已经是AB的混合结果）
            // 然后第二次混合时C的归一化权重是0.2/(0.2+0.8)，A占0.8。
            // 最终得到的结果跟我们平时计算0.5*A+0.3*B+0.2*C是一样的。
            m_weight.m_blend_weight[i] = sum_weight;
            
            bone_trans_one.m_position  = Vector3::lerp(bone_trans_one.m_position,bone_trans_two.m_position,cur_weight);
            bone_trans_one.m_scale     = Vector3::lerp(bone_trans_one.m_scale,bone_trans_two.m_scale,cur_weight);
            bone_trans_one.m_rotation  = Quaternion::nLerp(cur_weight,bone_trans_one.m_rotation,bone_trans_two.m_rotation,true);
        }
    }
}



