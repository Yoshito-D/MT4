#include <Novice.h>
#include "Math/Math.h"
const char kWindowTitle[] = "LE2A_20_ヨシトダイキ_01_01";

static const int kRowHeight = 20;
static const int kColumnWidth = 60;

static const float kWindowWidth = 1280.0f;
static const float kWindowHeight = 720.0f;

void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label);
void QuaternionScreenPrintf(int x, int y, const Quaternion& quaternion, const char* label);
void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label);

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

   // ライブラリの初期化
   Novice::Initialize(kWindowTitle, 1280, 720);

   // キー入力結果を受け取る箱
   char keys[256] = { 0 };
   char preKeys[256] = { 0 };

   Quaternion rotation = MakeRotateAxisAngleQuaternion(Vector3(1.0f,0.4f,-0.2f).Normalize(), 0.45f);
   Vector3 pointY = { 2.1f,-0.9f,1.3f };
   Matrix4x4 rotateMatrix = MakeRotateMatrix(rotation);
   Vector3 rotateByQuaternion = RotateVector(pointY, rotation);
   Vector3 rotateByMatrix = Transform(pointY, rotateMatrix);

   // ウィンドウの×ボタンが押されるまでループ
   while (Novice::ProcessMessage() == 0) {
	  // フレームの開始
	  Novice::BeginFrame();

	  // キー入力を受け取る
	  memcpy(preKeys, keys, 256);
	  Novice::GetHitKeyStateAll(keys);

	  ///
	  /// ↓更新処理ここから
	  ///

	  ///
	  /// ↑更新処理ここまで
	  ///

	  ///
	  /// ↓描画処理ここから
	  ///

	  QuaternionScreenPrintf(0, 0, rotation, " : rotation");
	  MatrixScreenPrintf(0, kRowHeight * 1, rotateMatrix, "rotateMatrix");
	  VectorScreenPrintf(0, kRowHeight * 6, rotateByQuaternion, " : rotateByQuaternion");
	  VectorScreenPrintf(0, kRowHeight * 7, rotateByMatrix, " : rotateByMatrix");

	  ///
	  /// ↑描画処理ここまで
	  ///

	  // フレームの終了
	  Novice::EndFrame();

	  // ESCキーが押されたらループを抜ける
	  if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
		 break;
	  }
   }

   // ライブラリの終了
   Novice::Finalize();
   return 0;
}

void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label) {
   Novice::ScreenPrintf(x, y, "%.02f", vector.x);
   Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", vector.y);
   Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", vector.z);
   Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%s", label);
}

void QuaternionScreenPrintf(int x, int y, const Quaternion& quaternion, const char* label) {
   Novice::ScreenPrintf(x, y, "%.02f", quaternion.x);
   Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", quaternion.y);
   Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", quaternion.z);
   Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%.02f", quaternion.w);
   Novice::ScreenPrintf(x + kColumnWidth * 4, y, "%s", label);
}

void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label) {
   Novice::ScreenPrintf(x, y, "%s", label);
   for (int row = 0; row < 4; ++row) {
	  for (int column = 0; column < 4; ++column) {
		 Novice::ScreenPrintf(x + column * kColumnWidth, y + row * kRowHeight + kRowHeight, "%6.03f", matrix.m[row][column]);
	  }
   }
}