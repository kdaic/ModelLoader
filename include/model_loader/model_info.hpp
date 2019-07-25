#ifndef MODEL_INFO_HPP_INCLUDE
#define MODEL_INFO_HPP_INCLUDE

#include <cstdlib>
#include <string>
#include <iostream>

#include <map>
#include <string>

#include <hrpModel/Body.h>
#include <hrpModel/Light.h>
#include <hrpModel/Sensor.h>

using namespace hrp;

struct TransformedShapeIndex {

  boost::array<double,12> transformMatrix;

  short inlinedShapeTransformMatrixIndex;

  short shapeIndex;
};

typedef std::vector<TransformedShapeIndex> TransformedShapeIndexSequence;
typedef std::vector<TransformedShapeIndexSequence> AllLinkShapeIndexSequence;

/// センサ情報を格納する構造体。
/// 旧IDLでは interface としていたが、新IDLでは struct となることに注意。
struct SensorInfo
{

  /// センサの種類を表す文字列。
  /// @details
  /// 現在のところ、以下が定義されている。
  /// - "Force"        - 6軸力センサ
  /// - "RateGyro"     - レートジャイロセンサ
  /// - "Acceleration" - 加速度センサ
  /// - "Vision"       - ビジョンセンサ
  /// - "Range"	     - 距離センサ
  std::string type;

  std::string name;             ///< 本センサの識別名
  long id;                       ///< センサの種類ごとのセンサID
  boost::array<double, 3> translation;         ///< センサ設置位置(リンクローカル座標)
  double rotation[4];            ///< センサ設置姿勢(リンクローカル座標)
  std::vector<float> specValues; ///< 各種仕様値(旧IDLのmaxValuesに相当)
  std::string specFile;          ///< 仕様記述ファイル名（本改良ではとりあえず空でよい）
  /// 本リンクに対応する形状情報の変換行列付きインデックス列
  TransformedShapeIndexSequence shapeIndices;
  std::vector<boost::array<double, 12> > inlinedShapeTransformMatrices;
};

typedef std::vector<SensorInfo> SensorInfoSequence;


/// ハードウェアコンポーネント情報を格納する構造体
struct HwcInfo {
  std::string name; ///< 本HWCの識別名
  long id;                 ///< HWCの種類ごとのセンサID
  boost::array<double,3> translation;   ///< HWC設置位置(リンクローカル座標)
  double rotation[4];      ///< HWC設置姿勢(リンクローカル座標)
  std::string url;         ///< HWCプロファイルのURL

  /// 本HWCに対応する形状情報の変換行列付きインデックス列
  TransformedShapeIndexSequence shapeIndices;
  std::vector<boost::array<double,12> > inlinedShapeTransformMatrices;
};
typedef std::vector<HwcInfo> HwcInfoSequence;


/// セグメントの情報を格納する構造体
/// 複数個のセグメントノードを持つリンクをGUIから編集するために使用
struct SegmentInfo {

  std::string name;    ///< セグメント名
  double mass;                ///< 質量
  boost::array<double,3> centerOfMass;     ///< 重心位置
  double inertia[9];          ///< 慣性行列
  double transformMatrix[12];
  /// TransformedShapeIndexのインデックス列
  TransformedShapeIndexSequence shapeIndices;
};

typedef std::vector<SegmentInfo> SegmentInfoSequence;

/// ライトの情報を格納する構造体
struct LightInfo {
  std::string name;            ///< 名称
  Light::LightType type;              ///< ライトの種類
  double transformMatrix[12];  ///< ライトの位置・姿勢（同時変換行列の上3行)
  double ambientIntensity;     // s, p, d
  double attenuation[3];       // s, p
  double color[3];             // s, p, d
  double intensity;            // s, p, d
  double location[3];          // s, p
  bool on;                     // s, p, d
  double radius;               // s, p
  double direction[3];         // s,    d
  double beamWidth;            // s
  double cutOffAngle;          // s
};

typedef std::vector<LightInfo> LightInfoSequence;

/// 各リンクの情報を格納する構造体。
/// 旧IDLでは interface としていたが、新IDLでは struct となることに注意。
/// equivalentInertia は廃止。
struct LinkInfo {
  std::string name;             ///< リンク名
  short jointId;                ///< 関節識別値
  std::string jointType; ///< 関節タイプ
  double jointValue;            ///< 関節初期値
  boost::array<double,3> jointAxis;          ///< 関節軸(リンクローカル座標)
  std::vector<double> ulimit;   ///< 最大関節値
  std::vector<double> llimit;   ///< 最小関節値
  std::vector<double> uvlimit;  ///< 最大関節速度値
  std::vector<double> lvlimit;  ///< 最小関節速度値
  std::vector<double> climit;   ///< 最大関節電流値 (tlimit = climit x grarRatio x torqueConst)
  double translation[3];        ///< ローカル座標系原点(親リンク相対)

  /// ローカル座標系姿勢(親リンク相対)
  /// 回転軸(x, y, z) + 回転角度の並びのサイズ４の配列(VRMLと同じ)
  double rotation[4];

  double mass;                     ///< 質量
  double centerOfMass[3];          ///< 重心位置
  double inertia[9];               ///< 慣性行列
  double rotorInertia;             ///< ロータ慣性
  double rotorResistor;            ///< ロータ抵抗
  double gearRatio;                ///< ギア比
  double torqueConst;              ///< トルク定数
  double encoderPulse;             ///< エンコーダパルス
  short parentIndex;               ///< 親リンクインデックス
  std::vector<short> childIndices; ///< 子リンクインデックス列
  /// 本リンクに対応する形状情報の変換行列付きインデックス列
  std::vector<TransformedShapeIndex> shapeIndices;
  /// 形状データのAABBtreeの階層の深さ＋１
  short AABBmaxDepth;
  /// 形状データのAABBtreeのBoundingBoxの最大個数
  short AABBmaxNum;
  /// shapeIndices の inlinedShapeTransformMatrixIndex によって指し示される行列リスト
  std::vector<boost::array<double,12> > inlinedShapeTransformMatrices;
  std::vector<SensorInfo> sensors;  ///< 本リンクに設置されたセンサの情報
  std::vector<HwcInfo> hwcs;
  std::vector<SegmentInfo> segments;
  std::vector<LightInfo> lights;

  /// アクチュエータ・ギア等の仕様記述ファイル名リスト
  /// （本改良ではとりあえず空でよい）
  std::vector<std::string> specFiles;
};

typedef std::vector<LinkInfo> LinkInfoSequence;


enum ShapePrimitiveType { SP_MESH, SP_BOX, SP_CYLINDER, SP_CONE, SP_SPHERE, SP_PLANE };

///  物体形状情報を格納する構造体。
struct ShapeInfo {
  /// 本ShapeがVRMLのinlineノード内に格納されている場合は、
  /// inlineされているVRMLファイルへのパスを格納する。
  /// inlineではなく直接メインのVRMLファイル内に形状が記述されていた場合は、
  /// 本フィールドは空とする。
  std::string url;

  /// オリジナルのVRMLモデルファイルにおけるプリミティブの種類を表す。
  /// クライアントは描画においてこの情報を利用することができる。
  /// ただし、primitiveType が MESH 以外のときも、プリミティブをメッシュに
  /// 展開した際の幾何データ(verticesなど)は持っているものとする。
  ShapePrimitiveType primitiveType;

  /// primitiveType が MESH 以外のとき、プリミティブの形状に関わるパラメータを格納する。
  /// 各プリミティブにおける配列要素とパラメータの対応は以下とする。
  ///
  /// - BOX
  ///   0?2: x, y, z のサイズ
  ///
  /// - CYLINDER
  ///   0: radius
  ///   1: height
  ///   2: top
  ///   3: bottom
  ///   4: side
  ///   bottom, side, top については値が0のときfalse, それ以外はtrueとする。
  ///   (CONEに関しても同様。）
  ///
  /// - CONE
  ///   0: bottomRadius
  ///      1: height
  ///      2: bottom
  ///      3: side
  ///
  /// - SPHERE
  ///   0: radius
  std::vector<float> primitiveParameters;

  /// 表面メッシュを構成する頂点データ。
  /// 連続する３要素が頂点位置の３次元ベクトルに対応する。
  std::vector<float> vertices;

  /// 表面メッシュを構成する三角形における頂点の組み合わせを格納したデータ。
  /// 各要素はverticesに格納された頂点のインデックスを表し、
  /// 連続する３要素によってメッシュを構成する三角形を指定する。
  /// メッシュ構成面は必ず三角形で表現されるものとする。
  ///
  /// なお、メッシュの表裏を区別する必要がある場合は、
  /// 連続する３要素が反時計回りとなる面を表とする。
  std::vector<long> triangles;

  /// 本Faceに対応するAppearanceInfoの
  /// BodyInfo::appearances におけるインデックス。
  long appearanceIndex;
};

typedef std::vector<ShapeInfo> ShapeInfoSequence;

/// 表面の見え情報を格納する構造体。
///
/// 本構造体の要素は、VRMLのIndexedFaceSetにみられるものと
/// 概ね同様である。
struct AppearanceInfo {
  /// 本Appearanceに対応するMaterialInfoがある場合、
  /// BodyInfo::materials におけるインデックス。
  /// なければ -1。
  long materialIndex;

  /// 法線データ。連続する３要素をx, y, zとし、一法線ベクトルに対応。
  /// この配列のサイズが0の場合、法線はクライアントが必要に応じて
  /// 生成しなければならない。
  std::vector<float> normals;

  /// 法線対応付けデータ。
  /// normalPerVertex が true なら、ShapeInfo の vertices の並びと対応させる。
  /// normalPerVertex が false なら、ShapeInfo における三角形ポリゴンの並びと対応させる。
  /// normalsがあってnormalIndicesのサイズが０の場合、normalsの要素を頂点または面に1対1対応させる。
  std::vector<long> normalIndices;

  bool normalPerVertex;

  bool solid;

  float creaseAngle;

  /// 色データ。連続する３要素をR,G,Bとし一色に対応。各要素の値の範囲は 0 から 1.0。
  /// この配列のサイズが0の場合、色はmaterialInfoのものになる。
  std::vector<float> colors;

  std::vector<long> colorIndices;

  /// 色対応付けデータ。
  /// colorPerVertex が true なら、ShapeInfo の vertices の並びと対応させる。
  /// colorPerVertex が false なら、ShapeInfo における三角形ポリゴンの並びと対応させる。
  /// colorsがあってcolorIndicesのサイズが０の場合、colorsの要素を頂点または面に1対1対応させる。
  bool colorPerVertex;

  /// テクスチャデータ。
  /// BodyInfo::textures におけるインデックス。
  /// 対応するテクスチャがなければ、-1。
  long textureIndex;

  std::vector<float> textureCoordinate;
  std::vector<long> textureCoordIndices;
  double textransformMatrix[9];
};

typedef std::vector<AppearanceInfo> AppearanceInfoSequence;


/// 表面材質情報を格納する構造体。
/// 各要素はVRMLのMaterialノードと同様。
/// 全ての変数の値の範囲は 0.0 〜 1.0。
struct MaterialInfo {

  float ambientIntensity;

  float diffuseColor[3];

  float emissiveColor[3];

  float shininess;

  float specularColor[3];

  float transparency;
};

typedef std::vector<MaterialInfo> MaterialInfoSequence;


/// テクスチャ情報を格納する構造体。
/// 各要素はVRMLのPixelTextureノードと同様。
struct TextureInfo {
  /// テクスチャの画像イメージ。
  /// VRMLのSFImageから、先頭の&lt;width&gt;, &lt;height&gt;, &lt;num components&gt;
  /// を除いたものと同様。&lt;width&gt;, &lt;height&gt;, &lt;num components&gt;に対応する
  /// 値は本構造体の width, height, numComponents で指定。
  ///
  /// 元のデータがurl指定の場合は、urlフィールドに画像ファイルの位置が格納される。
  /// この場合、モデルローダ側で画像の展開が行われなかった場合は、
  /// imageフィールドのサイズは0となっており、
  /// クライアントはファイル名からテクスチャを獲得する必要がある。
  /// image フィールドのサイズが 0 でなくて、urlのサイズも 0 でない場合は、
  /// クライアントは好きな方のやり方でテクスチャ画像を獲得すればよい。
  std::vector<unsigned char> image;
  short numComponents;
  short width;
  short height;
  bool repeatS;
  bool repeatT;
  std::string url;
};

typedef std::vector<TextureInfo> TextureInfoSequence;


/// 閉リンク機構のリンクの接続情報を格納する構造体。
struct ExtraJointInfo {
  ///  Extra Joint name
  std::string name;
  /// Possible types are "xyz","xy","z"
  Body::ExtraJointType jointType;
  /// Constraint axes. Two or three orthogonal axes must be specifiedin the local coordinate of the first link
  boost::array<double,3> axis;
  /// Name of the link
  std::string link[2];
  /// Connection (joint) position in the local coordinate of the link
  boost::array<double,3> point[2];
};

typedef std::vector<ExtraJointInfo> ExtraJointInfoSequence;


/// 画像フォーマット
enum PixelFormat {ARGB, // 4byte/pixel
                  GRAY, // 1byte/pixel
                  DEPTH,// 4byte/pixel
                  RGB}; // 3byte/pixel


/// 画像データ
struct ImageData
{
  /// フォーマット
  PixelFormat format;

  /// 画像の幅
  long width;

  /// 画像の高さ
  long height;

  /// octet での画像データ
  std::vector<unsigned char> octetData;

  /// long での画像データ
  std::vector<long> longData;

  /// float での画像データ
  std::vector<float> floatData;
};


/// カメラ
/// @author	Ichitaro Kohara, MSTC
/// @version	1.0(2001.02.16)
class Camera
{
public:
  /// カメラ種別
  enum CameraType {
    NONE, 		//!< read no information
    COLOR, 		//!< read color buffer
    MONO,
    DEPTH, 		//!< read depth buffer
    COLOR_DEPTH,	//!< read color buffer and depth buffer
    MONO_DEPTH
  };

  /// カメラパラメタ
  struct CameraParameter
  {
    /// カメラ種別
    CameraType	type;

    /// キャラクタの正面からの距離[m]
    float	frontClipDistance;

    /// キャラクタの後面までの距離[m]
    float	backClipDistance;

    /// 視野角[rad]
    float	fieldOfView;

    /// センサID
    long	sensorId;

    /// センサ名
    std::string	sensorName;

    /// ノード宣言の名前
    std::string	defName;

    /// 幅
    long	width;

    /// 高さ
    long	height;

    /// フレームレート[fps]
    float  frameRate;
  };

  /// サーバを終了します。
  void destroy();

  /// カメラパラメタを取得します。
  /// @return カメラパラメタ
  CameraParameter
  getCameraParameter();

  /// イメージを取得します。
  ImageData getImageData();
};

/// カメラ配列
typedef std::vector<Camera> CameraSequence;



enum AABBdataType { AABB_DEPTH, AABB_NUM };

struct ModelLoadOption {
  bool readImage;
  std::vector<short> AABBdata;
  AABBdataType AABBtype;
};


#endif // MODEL_INFO_HPP_INCLUDE
