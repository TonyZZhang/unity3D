using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class JointPoint
{
    //Landmark data
    public Vector3 LandmarkPose = new Vector3();
    public Vector3 WorldPos  = new Vector3();
    
    // Bones
    public Transform Transform = null;
    public Vector3 FilteredPos  = new Vector3();
    public Vector3[] LastPoses = new Vector3[6];
    public Quaternion InitRotation;
    public Quaternion Inverse;
    public Quaternion InverseRotation;
    public Vector3 InitialRotation;
    
    public JointPoint Child = null;
    public JointPoint Parent = null;

    public float DistanceFromChild;
    public float DistanceFromDad;
    
    
    // For Kalman filter
    public Vector3 P = new Vector3();
    public Vector3 X = new Vector3();
    public Vector3 K = new Vector3();
}

public class Anima : MonoBehaviour
{
    // Start is called before the first frame update
     public enum BodyPoints : int
    {
        Nose,
        LeftEyeInner,
        LeftEye,
        LeftEyeOuter,
        RightEyeInner,
        RightEye,
        RightEyeOuter,
        LeftEar,
        RightEar,
        LeftMouth,
        RightMouth,
        LeftShoulder,
        RightShoulder,
        LeftElbow,
        RightElbow,
        LeftWrist,
        RightWrist,
        LeftPinky,
        RightPinky,
        LeftIndex,
        RightIndex,
        LeftThump,
        RightThump,
        LeftHip,
        RightHip,
        LeftKnee,
        RightKnee,
        LeftAnkle,
        RightAnkle,
        LeftHeel,
        RightHeel,
        LeftFootIndex,
        RightFootIndex,
        Hips,
        Spine,
        Neck,
        Head
    }

    public struct CharacterBody
{
    public GameObject nose;
    public GameObject leftEyeInner;
    public GameObject leftEye;
    public GameObject leftEyeOuter;
    public GameObject rightEyeInner;
    public GameObject rightEye;
    public GameObject rightEyeOuter;
    public GameObject leftEar;
    public GameObject rightEar;
    public GameObject leftMouth;
    public GameObject rightMouth;

    public GameObject hips;
    public GameObject leftShoulder;
    public GameObject rightShoulder;
    public GameObject leftElbow;
    public GameObject rightElbow;
    public GameObject leftWrist;
    public GameObject rightWrist;
    public GameObject leftHip;
    public GameObject rightHip;
    public GameObject leftKnee;
    public GameObject rightKnee;
    public GameObject leftAnkle;
    public GameObject rightAnkle;
    public GameObject leftHeel;
    public GameObject rightHeel;
}

    public UDPRecive udpRecive;
    private Animator anim;
    private JointPoint[] jointPoints;
    private GameObject[] jointsDebug;

    private BodyPartVector[] poseVec;

    [Tooltip("optional")] [SerializeField] private CharacterBody characterBodyIK;
    private Transform hips;
    [SerializeField] private bool IKEnable;
    [SerializeField] private bool normalMode;
    [SerializeField] private Transform characterPlacement;
    private Vector3 headUpVector;
    private int frame;
    private Vector3 distanceOffset;

    
    public float fraction = 1.2f;
    public float fractionX = 1.2f;
    public float fractionY = 1.2f;
    public float fractionZ = 1.2f;
    [SerializeField] private bool enableVideoAspectRatioEffector;
    private float videoFractionX = 1;
    private float videoFractionY = 1;
    private float videoFractionZ = 1;

   [Serializable]
    public struct BodyPart
    {
        public float x;
        public float y;
        public float z;
        public float visibility;
    }

    [Serializable]
    public class PoseJson
    {
        public BodyPart[] predictions;
        public float width;
        public float height;
        public int frame;
        
    }

   [Serializable]
    public struct BodyPartVector
    {
        public Vector3 position;
        public float visibility;
    }

    [Serializable]
    public class PoseJsonVector
    {
        public BodyPartVector[] predictions;
        public float width;
        public float height;
        public int frame;

    }

    void Start()
    {
        poseVec = new BodyPartVector[37];
        frame = 0;
        jointPoints = new JointPoint[37];
        anim = this.GetComponent<Animator>();
        Init();
    }

    // Update is called once per frame
    void Update()
    {
        TestFile();
    }

    void Init()
    {
        //udpRecive = this.GetComponent<UDPRecive>();
        udpRecive.Test();
        jointPoints = new JointPoint[37];
        for (var i = 0; i < jointPoints.Length; i++)
        {
            jointPoints[i] = new JointPoint();

        }
        // Right Arm
        jointPoints[(int) BodyPoints.RightShoulder].Transform = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
        jointPoints[(int) BodyPoints.RightElbow].Transform = anim.GetBoneTransform(HumanBodyBones.RightLowerArm);
        jointPoints[(int) BodyPoints.RightWrist].Transform = anim.GetBoneTransform(HumanBodyBones.RightHand);
        // Left Arm
        jointPoints[(int) BodyPoints.LeftShoulder].Transform = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        jointPoints[(int) BodyPoints.LeftElbow].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        jointPoints[(int) BodyPoints.LeftWrist].Transform = anim.GetBoneTransform(HumanBodyBones.LeftHand);
        
        // Right Leg
        jointPoints[(int) BodyPoints.RightHip].Transform = anim.GetBoneTransform(HumanBodyBones.RightUpperLeg);
        jointPoints[(int) BodyPoints.RightKnee].Transform = anim.GetBoneTransform(HumanBodyBones.RightLowerLeg);
        jointPoints[(int) BodyPoints.RightAnkle].Transform = anim.GetBoneTransform(HumanBodyBones.RightFoot);
        jointPoints[(int) BodyPoints.RightFootIndex].Transform = anim.GetBoneTransform(HumanBodyBones.RightToes);

        // Left Leg
        jointPoints[(int) BodyPoints.LeftHip].Transform = anim.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
        jointPoints[(int) BodyPoints.LeftKnee].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
        jointPoints[(int) BodyPoints.LeftAnkle].Transform = anim.GetBoneTransform(HumanBodyBones.LeftFoot);
        jointPoints[(int) BodyPoints.LeftFootIndex].Transform = anim.GetBoneTransform(HumanBodyBones.LeftToes);

        // etc
        //jointPoints[PositionIndex.abdomenUpper.Int()].Transform = anim.GetBoneTransform(HumanBodyBones.Spine);
        // jointPoints[(int) BodyPoints.Hips].Transform = hips.transform;
        jointPoints[(int) BodyPoints.Hips].Transform = anim.GetBoneTransform(HumanBodyBones.Hips);
        jointPoints[(int) BodyPoints.Head].Transform = anim.GetBoneTransform(HumanBodyBones.Head);
        jointPoints[(int) BodyPoints.Neck].Transform = anim.GetBoneTransform(HumanBodyBones.Neck);
        jointPoints[(int) BodyPoints.Spine].Transform = anim.GetBoneTransform(HumanBodyBones.Spine);
        

        // Child Settings
        // Right Arm
        jointPoints[(int) BodyPoints.RightShoulder].Child = jointPoints[(int) BodyPoints.RightElbow];
        jointPoints[(int) BodyPoints.RightElbow].Child = jointPoints[(int) BodyPoints.RightWrist];
        jointPoints[(int) BodyPoints.RightElbow].Parent = jointPoints[(int) BodyPoints.RightShoulder];

        // Left Arm
        jointPoints[(int) BodyPoints.LeftShoulder].Child = jointPoints[(int) BodyPoints.LeftElbow];
        jointPoints[(int) BodyPoints.LeftElbow].Child = jointPoints[(int) BodyPoints.LeftWrist];
        jointPoints[(int) BodyPoints.LeftElbow].Parent = jointPoints[(int) BodyPoints.LeftShoulder];

        // Fase

        
        
        // Right Leg
        jointPoints[(int) BodyPoints.RightHip].Child = jointPoints[(int) BodyPoints.RightKnee];
        jointPoints[(int) BodyPoints.RightKnee].Child = jointPoints[(int) BodyPoints.RightAnkle];
        jointPoints[(int) BodyPoints.RightAnkle].Child = jointPoints[(int) BodyPoints.RightFootIndex];
        // jointPoints[(int) BodyPoints.RightKnee].Parent = jointPoints[(int) BodyPoints.RightHip];
        jointPoints[(int) BodyPoints.RightAnkle].Parent = jointPoints[(int) BodyPoints.RightKnee];

        // Left Leg
        jointPoints[(int) BodyPoints.LeftHip].Child = jointPoints[(int) BodyPoints.LeftKnee];
        jointPoints[(int) BodyPoints.LeftKnee].Child = jointPoints[(int) BodyPoints.LeftAnkle];
        jointPoints[(int) BodyPoints.LeftAnkle].Child = jointPoints[(int) BodyPoints.LeftFootIndex];
        // jointPoints[(int) BodyPoints.LeftKnee].Parent = jointPoints[(int) BodyPoints.LeftHip];
        jointPoints[(int) BodyPoints.LeftAnkle].Parent = jointPoints[(int) BodyPoints.LeftKnee];

        // etc
        // jointPoints[(int) BodyPoints.Spine].Child = jointPoints[(int) BodyPoints.Neck];
        // jointPoints[(int) BodyPoints.Neck].Child = jointPoints[(int) BodyPoints.Head];
        // jointPoints[(int) BodyPoints.Head].Child = jointPoints[(int) BodyPoints.Nose];

        
        for (int i = 0; i < jointPoints.Length; i++)
        {
            if (jointPoints[i].Child != null)
            {
                if(jointPoints[i].Child.Transform != null)            
                {
                    jointPoints[i].DistanceFromChild = Vector3.Distance(jointPoints[i].Child.Transform.position,
                        jointPoints[i].Transform.position);
                }
            }
        }
        

        // Set Inverse
        Vector3 a = jointPoints[(int) BodyPoints.LeftHip].Transform.position;
        Vector3 b = jointPoints[(int) BodyPoints.Spine].Transform.position;
        hips = jointPoints[(int) BodyPoints.Hips].Transform;
        Vector3 c = jointPoints[(int) BodyPoints.RightHip].Transform.position;
        var forward = b.TriangleNormal(a,c);
        
        
        
        foreach (var jointPoint in jointPoints)
        {
            if (jointPoint.Transform != null)
            {
                jointPoint.InitRotation = jointPoint.Transform.rotation;
            }

            if (jointPoint.Child != null)
            {
                jointPoint.Inverse = Quaternion.Inverse(Quaternion.LookRotation(jointPoint.Transform.position - jointPoint.Child.Transform.position, forward));
                jointPoint.InverseRotation = jointPoint.Inverse * jointPoint.InitRotation;
            }
        }
        //Hip and Spine
        var hip = jointPoints[(int) BodyPoints.Hips];
        var spine = jointPoints[(int) BodyPoints.Spine];
        hip.Inverse = Quaternion.Inverse(Quaternion.LookRotation(forward,spine.Transform.position-hip.Transform.position));
        hip.InverseRotation = hip.Inverse * hip.InitRotation;

        if (spine.Transform != null)
        {
            spine.Inverse = Quaternion.Inverse(Quaternion.LookRotation(
                spine.Transform.position.TriangleNormal(jointPoints[(int) BodyPoints.RightShoulder].Transform.position,
                    jointPoints[(int) BodyPoints.LeftShoulder].Transform.position),
                jointPoints[(int) BodyPoints.Neck].Transform.position - spine.Transform.position));
            spine.InverseRotation = spine.Inverse * spine.InitRotation;
        }

        // For Head Rotation
        var head = jointPoints[(int) BodyPoints.Head];
        head.InitRotation = jointPoints[(int) BodyPoints.Head].Transform.rotation;
        var gaze = head.Transform.up;
        Debug.Log(gaze);
        head.Inverse = Quaternion.Inverse(Quaternion.LookRotation(gaze));
       // head.InverseRotation = head.Inverse * head.InitRotation; //TODO check why?
       
        head.InverseRotation = head.InitRotation;
        headUpVector = head.Transform.up;
        
        
        //feet setup
        var r_feet = jointPoints[(int) BodyPoints.RightAnkle];
        
        r_feet.Inverse = Quaternion.Inverse(Quaternion.LookRotation(r_feet.Transform.position - jointPoints[(int) BodyPoints.RightFootIndex].Transform.position, jointPoints[(int) BodyPoints.RightKnee].Transform.position - r_feet.Transform.position));
        r_feet.InverseRotation = r_feet.Inverse * r_feet.InitRotation;
        
        var l_feet = jointPoints[(int) BodyPoints.LeftAnkle];
        l_feet.Inverse = Quaternion.Inverse(Quaternion.LookRotation(l_feet.Transform.position - jointPoints[(int) BodyPoints.LeftFootIndex].Transform.position, jointPoints[(int) BodyPoints.LeftKnee].Transform.position - l_feet.Transform.position));
        l_feet.InverseRotation = l_feet.Inverse * l_feet.InitRotation;
    }

    private void UpdateNormalMode(BodyPartVector[] bodyPartVectors)
    {

        for (int i = 0; i < bodyPartVectors.Length; i++)
        {
            jointPoints[i].LandmarkPose = bodyPartVectors[i].position;
        }
        
        //setting position of each bone
        // jointPoints[(int) BodyPoints.Spine].Transform.position = bodyPartVectors[(int) BodyPoints.Spine].position;
        jointPoints[(int) BodyPoints.Hips].Transform.position = bodyPartVectors[(int) BodyPoints.Hips].position;

        for (int i = 0; i < jointPoints.Length && i < bodyPartVectors.Length; i++)
        {
            JointPoint bone = jointPoints[i];

            if (bone.Transform != null)
                bone.WorldPos = bone.Transform.position;
        }



        for (int i = 0; i < jointPoints.Length && i < bodyPartVectors.Length; i++)
        {
            JointPoint bone = jointPoints[i];
            
            if (bone.Child != null)
            {
                if (bone.Child.Transform != null) 
                {
                    JointPoint child = bone.Child;
                    float distance = bone.DistanceFromChild;
                    Vector3 direction = (-bone.LandmarkPose + child.LandmarkPose) / (-bone.LandmarkPose + child.LandmarkPose).magnitude;
                    child.WorldPos = bone.Transform.position + direction * distance;
                    // child.Transform.position = child.WorldPos;
//                    Debug.Log(distance + "  " + Vector3.Distance(child.Transform.position,bone.Transform.position));
                }
            }
        }

        for (int i = 0; i < jointPoints.Length; i++)
        {
                jointPoints[i].FilteredPos = jointPoints[i].WorldPos;
        }

        //setting hip & spine rotation
        Vector3 a = bodyPartVectors[(int) BodyPoints.RightHip].position;
        Vector3 spine = bodyPartVectors[(int) BodyPoints.Spine].position;
        Vector3 hip = bodyPartVectors[(int) BodyPoints.Hips].position;
        Vector3 c = bodyPartVectors[(int) BodyPoints.LeftHip].position;
        Vector3 d = bodyPartVectors[(int) BodyPoints.RightShoulder].position;
        Vector3 e = bodyPartVectors[(int) BodyPoints.LeftShoulder].position;
        Vector3 hipsUpward = spine - hip;
        Vector3 spineUpward = bodyPartVectors[(int) BodyPoints.Neck].position - spine;
        jointPoints[(int) BodyPoints.Hips].Transform.rotation = Quaternion.LookRotation(spine.TriangleNormal(c, a),
                                                                   hipsUpward ) *
                                                                jointPoints[(int) BodyPoints.Hips].InverseRotation;

        jointPoints[(int) BodyPoints.Spine].Transform.rotation = Quaternion.LookRotation(spine.TriangleNormal(d, e),
                                                                     spineUpward ) *
                                                                jointPoints[(int) BodyPoints.Spine].InverseRotation;

        
        // Head Rotation
        Vector3 mouth = (bodyPartVectors[(int) BodyPoints.LeftMouth].position +
                         bodyPartVectors[(int) BodyPoints.RightMouth].position)/2.0f;
        Vector3 lEye = bodyPartVectors[(int) BodyPoints.LeftEye].position;
        Vector3 rEye = bodyPartVectors[(int) BodyPoints.RightEye].position;
                
        var gaze = lEye.TriangleNormal(mouth, rEye);
        
        Vector3 nose = bodyPartVectors[(int) BodyPoints.Nose].position;
        Vector3 rEar = bodyPartVectors[(int) BodyPoints.RightEar].position;
        Vector3 lEar = bodyPartVectors[(int) BodyPoints.LeftEar].position;
        var head = jointPoints[(int) BodyPoints.Head];
        Vector3 normal = nose.TriangleNormal(rEar, lEar);
        head.Transform.rotation = Quaternion.LookRotation(gaze, normal) * head.InverseRotation;
        
        
        // rotate each of bones
        Vector3 forward = jointPoints[(int) BodyPoints.Hips].Transform.forward;
        
        Vector3 leftHip = jointPoints[(int) BodyPoints.LeftHip].FilteredPos;
        Vector3 rightHip = jointPoints[(int) BodyPoints.RightHip].FilteredPos;
        forward = jointPoints[(int) BodyPoints.Spine].FilteredPos.TriangleNormal(leftHip,rightHip);

        
        foreach (var jointPoint in jointPoints)
        {
            if(jointPoint == null)
                continue;
            
            if (jointPoint.Parent != null)
            {
                Vector3 fv = jointPoint.Parent.FilteredPos - jointPoint.FilteredPos;
                jointPoint.Transform.rotation = 
                    Quaternion.LookRotation(jointPoint.FilteredPos- jointPoint.Child.FilteredPos, fv) 
                    * jointPoint.InverseRotation;
            }
            else if (jointPoint.Child != null)
            {
                jointPoint.Transform.rotation = 
                    Quaternion.LookRotation((jointPoint.FilteredPos- jointPoint.Child.FilteredPos).normalized, forward)
                    * jointPoint.InverseRotation;
            }
            continue;
            
            if (jointPoint.Parent != null)
            {
                Vector3 fv = jointPoint.Parent.Transform.position - jointPoint.Transform.position;
                jointPoint.Transform.rotation = Quaternion.LookRotation(jointPoint.Transform.position- jointPoint.Child.Transform.position, fv) * jointPoint.InverseRotation;
            }
            else if (jointPoint.Child != null)
            {
                jointPoint.Transform.rotation = Quaternion.LookRotation(jointPoint.Transform.position- jointPoint.Child.Transform.position, forward) * jointPoint.InverseRotation;
            }
            continue;
            if (jointPoint.Parent != null)
            {
                var fv = jointPoint.Parent.Transform.position - jointPoint.Transform.position;
                jointPoint.Transform.rotation = Quaternion.LookRotation(jointPoint.Transform.position- jointPoint.Child.Transform.position, fv);
            }
            else if (jointPoint.Child != null)
            {
                jointPoint.Transform.rotation = Quaternion.LookRotation(jointPoint.Transform.position- jointPoint.Child.Transform.position, forward);
            }
        }

        
        //Calculate feet rotation
        Vector3 r_ankle = bodyPartVectors[(int) BodyPoints.RightAnkle].position;
        Vector3 r_toe = bodyPartVectors[(int) BodyPoints.RightFootIndex].position;
        Vector3 r_knee = bodyPartVectors[(int) BodyPoints.RightKnee].position;
        
        JointPoint r_ankleT = jointPoints[(int) BodyPoints.RightAnkle];
        r_ankleT.Transform.rotation = 
            Quaternion.LookRotation(r_ankle - r_toe, r_knee - r_ankle) 
            * r_ankleT.InverseRotation;
        
        Vector3 l_ankle = bodyPartVectors[(int) BodyPoints.LeftAnkle].position;
        Vector3 l_toe = bodyPartVectors[(int) BodyPoints.LeftFootIndex].position;
        Vector3 l_knee = bodyPartVectors[(int) BodyPoints.LeftKnee].position;
        
        JointPoint l_ankleT = jointPoints[(int) BodyPoints.LeftAnkle];
        l_ankleT.Transform.rotation = 
            Quaternion.LookRotation(l_ankle - l_toe, l_knee - l_ankle) 
            * l_ankleT.InverseRotation;
    }

    private T GetBodyParts<T>(string jsonText)
    {
        return JsonUtility.FromJson<T>(jsonText);
    }
    private PoseJsonVector GetBodyPartsVector(PoseJson poseJson)
    {
        Debug.Log(poseJson.predictions);
        int len = poseJson.predictions.Length;
        PoseJsonVector poseJsonVector = new PoseJsonVector();
        poseJsonVector.predictions = new BodyPartVector[len];
        poseJsonVector.frame = poseJson.frame;
        poseJsonVector.width = poseJson.width;
        poseJsonVector.height = poseJson.height;
        for (int i = 0; i < len; i++)
        {
            poseJsonVector.predictions[i].position = fraction * new Vector3(-poseJson.predictions[i].x*fractionX * videoFractionX,
                -poseJson.predictions[i].y*fractionY*videoFractionY,poseJson.predictions[i].z*fractionZ*videoFractionZ);
            poseJsonVector.predictions[i].visibility = poseJson.predictions[i].visibility;
            
        }

        return poseJsonVector;
    }

    void TestFile() 
    {

        string jsonTest;
        PoseJson currentPoseJson;
        PoseJsonVector currentPoseJsonVector;
        PoseJsonVector currentPoseJsonVectorNew;
        // StreamReader reader = new StreamReader("D:\\workspace\\3d\\pose.json");
        // jsonTest = reader.ReadToEnd();
        jsonTest = udpRecive.data;
        //Debug.Log(jsonTest);
        if (jsonTest != "") {
            currentPoseJson = GetBodyParts<PoseJson>(jsonTest);
            Debug.Log(currentPoseJson.predictions);
            Debug.Log(currentPoseJson.width);
            currentPoseJsonVector = GetBodyPartsVector(currentPoseJson);
            UpdateNormalMode(currentPoseJsonVector.predictions);
        }
    }
}