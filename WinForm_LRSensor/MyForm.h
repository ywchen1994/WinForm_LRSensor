#pragma once
#define _USE_MATH_DEFINES
#include <Windows.h>

#include <fstream>
#include<vector>
#include"Pt.h"
#include "CTBox.h"
#include"opencv2\opencv.hpp"
#include"..\DeviceEnum\DeviceEnumerator.h"
#include<time.h>
#include<utility>

namespace WinForm_LRSensor {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO;
	using namespace System::IO::Ports;
	using namespace System::Runtime::InteropServices;
	using namespace cv;
	using namespace std;
	double LIDAR_X_cooridate[361] = { 0 };
	double LIDAR_Y_cooridate[361] = { 0 };
	double LIDAR_R_cooridate[361] = { 0 };
	uint ComPortNoRecord[3] = { 0 };
	uint counter = 0;
	fstream ConnectRecord;
	VideoCapture cap;
	DeviceEnumerator de;
	vector<uint> RadarAlertIndex;
	vector<Pt>Pt_oldClusterRefPoint;
	std::map<int, Device> devices = de.getVideoDevicesMap();
	uint format = 25;
	double bsdAngle = 0;
	Pt AngleRadar_Point;
	float AlphaBias;

	int LiDAR_Data[722] = { 0 };
	CTBox TBox;
	double PartitionValue = 0;
	Pt left_Radar_bias;
	Pt right_Radar_bias;
	Radar RadarData;
	std::string FileNameTime;
	std::string LoadFilePath;
	int f_model_changed = 0;
	VideoWriter videoWrite;
	RNG rng(12345);
	time_t t1;
	fstream fp_LiDarReader, fp_TBoxReader;
	vector<tuple<Pt, Pt,Mat>> StatisticPt;
	
	
	
	/// <summary>
	/// MyForm 的摘要 
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO:  在此加入建構函式程式碼
			timer1->Interval = 10;
			timer1->Start();
			ComPortRefresh();
			chart1->Show();
			LoadData();
			LoadComPort();
			

		}

	protected:
		/// <summary>
		/// 清除任何使用中的資源。
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
		/// <summary>
		/// 設計工具所需的變數。
		/// </summary>
		double CurrentSpeed = 0;
		uint LiDARbufferIndex = 0;
		bool f_getLiDARData = false;
		bool f_getRRadarBias;
		bool f_getHeader = false;
#pragma region Design Element
	private: System::Windows::Forms::TabControl^  tabControl1;
	private: System::Windows::Forms::TabPage^  tabPage1;
	private: System::Windows::Forms::PictureBox^  pictureBox1;
	private: System::Windows::Forms::TabPage^  tabPage8;
	private: System::IO::Ports::SerialPort^  serialPort_LiDAR;
	private: System::ComponentModel::IContainer^  components;
	private: System::Windows::Forms::Timer^  timer1;
	private: System::IO::Ports::SerialPort^  serialPort_Radar;
	private: System::IO::Ports::SerialPort^  serialPort_Tbox;
	private: System::Windows::Forms::Label^  tx_TBox_LAngle;
	private: System::Windows::Forms::Label^  tx_TBox_RAngle;
	private: System::Windows::Forms::Label^  Tx_CarSpeed;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chart1;
	private: System::Windows::Forms::TabPage^  tabPage2;
	private: System::Windows::Forms::Button^  btn_RecordCnt;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Button^  Btn_CamCnt;
	private: System::Windows::Forms::ComboBox^  cBox_CameraList;
	private: System::Windows::Forms::GroupBox^  groupBox5;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::Button^  Btn_Radar_Connect;
	private: System::Windows::Forms::ComboBox^  cBox_Radar;
	private: System::Windows::Forms::Button^  Btn_Refresh_Combox;
	private: System::Windows::Forms::Button^  Btn_UpDateSetting;
	private: System::Windows::Forms::GroupBox^  groupBox9;
	private: System::Windows::Forms::Label^  label21;
	private: System::Windows::Forms::ComboBox^  cBox_LiDAR;
	private: System::Windows::Forms::Label^  label22;
	private: System::Windows::Forms::TextBox^  tBox_Partition;
	private: System::Windows::Forms::Button^  Btn_LiDARCnt;
	private: System::Windows::Forms::Button^  Btn_LiDARClose;
	private: System::Windows::Forms::GroupBox^  groupBox6;




	private: System::Windows::Forms::Button^  Btn_TboxClose;
	private: System::Windows::Forms::Label^  Tx_Radar_Mode;
	private: System::Windows::Forms::ComboBox^  cBox_TBox;
	private: System::Windows::Forms::Button^  Btn_TboxCnt;








	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chart2;
	private: System::Windows::Forms::Button^  Btn_PlayPause;
	private: System::Windows::Forms::PictureBox^  pictureBox2;
	private: System::Windows::Forms::Timer^  timer2;
	private: System::Windows::Forms::Label^  Tx_CarSpeed2;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Button^  Btn_UpDateFileName;
	private: System::Windows::Forms::Button^  Btn_LoadFilePath;
	private: System::Windows::Forms::TextBox^  tBox_LoadPath;
	private: System::Windows::Forms::TrackBar^  trackBar1;
	private: System::Windows::Forms::Label^  Tx_MaxFrame;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Button^  Btn_SpeedUp;
	private: System::Windows::Forms::Button^  Btn_SlowDown;
	private: System::Windows::Forms::CheckBox^  cBox_ScanPt;
	private: System::Windows::Forms::Label^  Tx_TBox_mode;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  tx_TBox_RDataP3;
	private: System::Windows::Forms::Label^  tx_TBox_LDataP3;
	private: System::Windows::Forms::CheckBox^  cBox_ShowLiText;
	private: System::Windows::Forms::Label^  Tx_BarPos;
	private: System::Windows::Forms::CheckBox^  cBox_ShowResult;
private: System::Windows::Forms::NumericUpDown^  numericUpDown1;
private: System::Windows::Forms::Label^  Tx_Result_LIDAR;
private: System::Windows::Forms::Label^  Tx_Result_Radar;





#pragma endregion
#pragma region Windows Form Designer generated code
			 /// <summary>
			 /// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改
			 /// 這個方法的內容。
			 /// </summary>
			 void InitializeComponent(void)
			 {
				 this->components = (gcnew System::ComponentModel::Container());
				 System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea3 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
				 System::Windows::Forms::DataVisualization::Charting::Legend^  legend3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series11 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series12 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series13 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series14 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series15 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series16 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(MyForm::typeid));
				 System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea4 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
				 System::Windows::Forms::DataVisualization::Charting::Legend^  legend4 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series17 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series18 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series19 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series20 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 this->tabControl1 = (gcnew System::Windows::Forms::TabControl());
				 this->tabPage1 = (gcnew System::Windows::Forms::TabPage());
				 this->Btn_UpDateFileName = (gcnew System::Windows::Forms::Button());
				 this->label7 = (gcnew System::Windows::Forms::Label());
				 this->chart1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
				 this->Tx_CarSpeed = (gcnew System::Windows::Forms::Label());
				 this->tx_TBox_LAngle = (gcnew System::Windows::Forms::Label());
				 this->tx_TBox_RAngle = (gcnew System::Windows::Forms::Label());
				 this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
				 this->tabPage2 = (gcnew System::Windows::Forms::TabPage());
				 this->btn_RecordCnt = (gcnew System::Windows::Forms::Button());
				 this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
				 this->Btn_CamCnt = (gcnew System::Windows::Forms::Button());
				 this->cBox_CameraList = (gcnew System::Windows::Forms::ComboBox());
				 this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
				 this->label9 = (gcnew System::Windows::Forms::Label());
				 this->Btn_Radar_Connect = (gcnew System::Windows::Forms::Button());
				 this->cBox_Radar = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_Refresh_Combox = (gcnew System::Windows::Forms::Button());
				 this->groupBox9 = (gcnew System::Windows::Forms::GroupBox());
				 this->Btn_UpDateSetting = (gcnew System::Windows::Forms::Button());
				 this->label1 = (gcnew System::Windows::Forms::Label());
				 this->label21 = (gcnew System::Windows::Forms::Label());
				 this->cBox_LiDAR = (gcnew System::Windows::Forms::ComboBox());
				 this->label22 = (gcnew System::Windows::Forms::Label());
				 this->tBox_Partition = (gcnew System::Windows::Forms::TextBox());
				 this->Btn_LiDARCnt = (gcnew System::Windows::Forms::Button());
				 this->Btn_LiDARClose = (gcnew System::Windows::Forms::Button());
				 this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
				 this->Btn_TboxClose = (gcnew System::Windows::Forms::Button());
				 this->Tx_Radar_Mode = (gcnew System::Windows::Forms::Label());
				 this->cBox_TBox = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_TboxCnt = (gcnew System::Windows::Forms::Button());
				 this->tabPage8 = (gcnew System::Windows::Forms::TabPage());
				 this->Tx_Result_Radar = (gcnew System::Windows::Forms::Label());
				 this->Tx_Result_LIDAR = (gcnew System::Windows::Forms::Label());
				 this->numericUpDown1 = (gcnew System::Windows::Forms::NumericUpDown());
				 this->cBox_ShowResult = (gcnew System::Windows::Forms::CheckBox());
				 this->Tx_BarPos = (gcnew System::Windows::Forms::Label());
				 this->cBox_ShowLiText = (gcnew System::Windows::Forms::CheckBox());
				 this->tx_TBox_RDataP3 = (gcnew System::Windows::Forms::Label());
				 this->tx_TBox_LDataP3 = (gcnew System::Windows::Forms::Label());
				 this->Tx_TBox_mode = (gcnew System::Windows::Forms::Label());
				 this->label3 = (gcnew System::Windows::Forms::Label());
				 this->cBox_ScanPt = (gcnew System::Windows::Forms::CheckBox());
				 this->Btn_SpeedUp = (gcnew System::Windows::Forms::Button());
				 this->Btn_SlowDown = (gcnew System::Windows::Forms::Button());
				 this->Tx_MaxFrame = (gcnew System::Windows::Forms::Label());
				 this->label2 = (gcnew System::Windows::Forms::Label());
				 this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
				 this->tBox_LoadPath = (gcnew System::Windows::Forms::TextBox());
				 this->Btn_LoadFilePath = (gcnew System::Windows::Forms::Button());
				 this->Tx_CarSpeed2 = (gcnew System::Windows::Forms::Label());
				 this->Btn_PlayPause = (gcnew System::Windows::Forms::Button());
				 this->pictureBox2 = (gcnew System::Windows::Forms::PictureBox());
				 this->chart2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
				 this->serialPort_LiDAR = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
				 this->serialPort_Radar = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->serialPort_Tbox = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->timer2 = (gcnew System::Windows::Forms::Timer(this->components));
				 this->tabControl1->SuspendLayout();
				 this->tabPage1->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
				 this->tabPage2->SuspendLayout();
				 this->groupBox1->SuspendLayout();
				 this->groupBox5->SuspendLayout();
				 this->groupBox9->SuspendLayout();
				 this->groupBox6->SuspendLayout();
				 this->tabPage8->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown1))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->BeginInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart2))->BeginInit();
				 this->SuspendLayout();
				 // 
				 // tabControl1
				 // 
				 this->tabControl1->Controls->Add(this->tabPage1);
				 this->tabControl1->Controls->Add(this->tabPage2);
				 this->tabControl1->Controls->Add(this->tabPage8);
				 this->tabControl1->Font = (gcnew System::Drawing::Font(L"新細明體", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(136)));
				 this->tabControl1->Location = System::Drawing::Point(12, -1);
				 this->tabControl1->Name = L"tabControl1";
				 this->tabControl1->SelectedIndex = 0;
				 this->tabControl1->Size = System::Drawing::Size(1890, 1000);
				 this->tabControl1->TabIndex = 0;
				 // 
				 // tabPage1
				 // 
				 this->tabPage1->BackColor = System::Drawing::Color::Transparent;
				 this->tabPage1->Controls->Add(this->Btn_UpDateFileName);
				 this->tabPage1->Controls->Add(this->label7);
				 this->tabPage1->Controls->Add(this->chart1);
				 this->tabPage1->Controls->Add(this->Tx_CarSpeed);
				 this->tabPage1->Controls->Add(this->tx_TBox_LAngle);
				 this->tabPage1->Controls->Add(this->tx_TBox_RAngle);
				 this->tabPage1->Controls->Add(this->pictureBox1);
				 this->tabPage1->Location = System::Drawing::Point(4, 22);
				 this->tabPage1->Name = L"tabPage1";
				 this->tabPage1->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage1->Size = System::Drawing::Size(1882, 974);
				 this->tabPage1->TabIndex = 0;
				 this->tabPage1->Text = L"數據";
				 // 
				 // Btn_UpDateFileName
				 // 
				 this->Btn_UpDateFileName->Location = System::Drawing::Point(1647, 460);
				 this->Btn_UpDateFileName->Name = L"Btn_UpDateFileName";
				 this->Btn_UpDateFileName->Size = System::Drawing::Size(129, 73);
				 this->Btn_UpDateFileName->TabIndex = 12;
				 this->Btn_UpDateFileName->Text = L"更新存檔名";
				 this->Btn_UpDateFileName->UseVisualStyleBackColor = true;
				 this->Btn_UpDateFileName->Click += gcnew System::EventHandler(this, &MyForm::Btn_UpDateFileName_Click);
				 // 
				 // label7
				 // 
				 this->label7->AutoSize = true;
				 this->label7->Location = System::Drawing::Point(1530, 665);
				 this->label7->Name = L"label7";
				 this->label7->Size = System::Drawing::Size(53, 12);
				 this->label7->TabIndex = 11;
				 this->label7->Text = L"CarSpeed:";
				 // 
				 // chart1
				 // 
				 chartArea3->AxisX->Interval = 100;
				 chartArea3->AxisX->IsReversed = true;
				 chartArea3->AxisX->Maximum = 1000;
				 chartArea3->AxisX->Minimum = -1000;
				 chartArea3->AxisY->Interval = 100;
				 chartArea3->AxisY->IsReversed = true;
				 chartArea3->AxisY->Maximum = 4000;
				 chartArea3->AxisY->Minimum = 0;
				 chartArea3->Name = L"ChartArea1";
				 this->chart1->ChartAreas->Add(chartArea3);
				 legend3->Name = L"Legend1";
				 this->chart1->Legends->Add(legend3);
				 this->chart1->Location = System::Drawing::Point(-42, -32);
				 this->chart1->Name = L"chart1";
				 series11->ChartArea = L"ChartArea1";
				 series11->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series11->Color = System::Drawing::Color::Khaki;
				 series11->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 series11->LabelForeColor = System::Drawing::Color::YellowGreen;
				 series11->Legend = L"Legend1";
				 series11->Name = L"Series_LiDAR";
				 series12->ChartArea = L"ChartArea1";
				 series12->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series12->Color = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)),
					 static_cast<System::Int32>(static_cast<System::Byte>(192)));
				 series12->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 series12->Legend = L"Legend1";
				 series12->MarkerColor = System::Drawing::Color::Blue;
				 series12->MarkerSize = 10;
				 series12->Name = L"Series_LiDAR_CLOSE";
				 series13->ChartArea = L"ChartArea1";
				 series13->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::FastPoint;
				 series13->Color = System::Drawing::Color::ForestGreen;
				 series13->Legend = L"Legend1";
				 series13->MarkerSize = 10;
				 series13->Name = L"Series_Radar_Angle";
				 series14->ChartArea = L"ChartArea1";
				 series14->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series14->Legend = L"Legend1";
				 series14->MarkerColor = System::Drawing::Color::DarkMagenta;
				 series14->MarkerSize = 15;
				 series14->MarkerStyle = System::Windows::Forms::DataVisualization::Charting::MarkerStyle::Star4;
				 series14->Name = L"Series_TBox_LRadar";
				 series15->ChartArea = L"ChartArea1";
				 series15->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series15->Color = System::Drawing::Color::Black;
				 series15->Legend = L"Legend1";
				 series15->MarkerSize = 10;
				 series15->Name = L"Series_TBox_RRadar";
				 series16->ChartArea = L"ChartArea1";
				 series16->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series16->Legend = L"Legend1";
				 series16->MarkerSize = 10;
				 series16->Name = L"Series_RadarDetectArea";
				 this->chart1->Series->Add(series11);
				 this->chart1->Series->Add(series12);
				 this->chart1->Series->Add(series13);
				 this->chart1->Series->Add(series14);
				 this->chart1->Series->Add(series15);
				 this->chart1->Series->Add(series16);
				 this->chart1->Size = System::Drawing::Size(1439, 1000);
				 this->chart1->TabIndex = 10;
				 this->chart1->Text = L"圖";
				 // 
				 // Tx_CarSpeed
				 // 
				 this->Tx_CarSpeed->AutoSize = true;
				 this->Tx_CarSpeed->Location = System::Drawing::Point(1589, 665);
				 this->Tx_CarSpeed->Name = L"Tx_CarSpeed";
				 this->Tx_CarSpeed->Size = System::Drawing::Size(33, 12);
				 this->Tx_CarSpeed->TabIndex = 3;
				 this->Tx_CarSpeed->Text = L"label7";
				 // 
				 // tx_TBox_LAngle
				 // 
				 this->tx_TBox_LAngle->AutoSize = true;
				 this->tx_TBox_LAngle->Location = System::Drawing::Point(1417, 601);
				 this->tx_TBox_LAngle->Name = L"tx_TBox_LAngle";
				 this->tx_TBox_LAngle->Size = System::Drawing::Size(33, 12);
				 this->tx_TBox_LAngle->TabIndex = 3;
				 this->tx_TBox_LAngle->Text = L"label7";
				 // 
				 // tx_TBox_RAngle
				 // 
				 this->tx_TBox_RAngle->AutoSize = true;
				 this->tx_TBox_RAngle->Location = System::Drawing::Point(1651, 601);
				 this->tx_TBox_RAngle->Name = L"tx_TBox_RAngle";
				 this->tx_TBox_RAngle->Size = System::Drawing::Size(33, 12);
				 this->tx_TBox_RAngle->TabIndex = 2;
				 this->tx_TBox_RAngle->Text = L"label7";
				 // 
				 // pictureBox1
				 // 
				 this->pictureBox1->Location = System::Drawing::Point(1359, 0);
				 this->pictureBox1->Name = L"pictureBox1";
				 this->pictureBox1->Size = System::Drawing::Size(487, 384);
				 this->pictureBox1->TabIndex = 1;
				 this->pictureBox1->TabStop = false;
				 // 
				 // tabPage2
				 // 
				 this->tabPage2->BackColor = System::Drawing::Color::Transparent;
				 this->tabPage2->Controls->Add(this->btn_RecordCnt);
				 this->tabPage2->Controls->Add(this->groupBox1);
				 this->tabPage2->Controls->Add(this->groupBox5);
				 this->tabPage2->Controls->Add(this->Btn_Refresh_Combox);
				 this->tabPage2->Controls->Add(this->groupBox9);
				 this->tabPage2->Controls->Add(this->groupBox6);
				 this->tabPage2->Font = (gcnew System::Drawing::Font(L"細明體", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(136)));
				 this->tabPage2->Location = System::Drawing::Point(4, 22);
				 this->tabPage2->Name = L"tabPage2";
				 this->tabPage2->Padding = System::Windows::Forms::Padding(3);
				 this->tabPage2->Size = System::Drawing::Size(1882, 974);
				 this->tabPage2->TabIndex = 1;
				 this->tabPage2->Text = L"設定";
				 // 
				 // btn_RecordCnt
				 // 
				 this->btn_RecordCnt->Location = System::Drawing::Point(249, 577);
				 this->btn_RecordCnt->Name = L"btn_RecordCnt";
				 this->btn_RecordCnt->Size = System::Drawing::Size(221, 106);
				 this->btn_RecordCnt->TabIndex = 15;
				 this->btn_RecordCnt->Text = L"Record Connect";
				 this->btn_RecordCnt->UseVisualStyleBackColor = true;
				 this->btn_RecordCnt->Click += gcnew System::EventHandler(this, &MyForm::btn_RecordCnt_Click);
				 // 
				 // groupBox1
				 // 
				 this->groupBox1->Controls->Add(this->Btn_CamCnt);
				 this->groupBox1->Controls->Add(this->cBox_CameraList);
				 this->groupBox1->Location = System::Drawing::Point(293, 120);
				 this->groupBox1->Name = L"groupBox1";
				 this->groupBox1->Size = System::Drawing::Size(150, 139);
				 this->groupBox1->TabIndex = 14;
				 this->groupBox1->TabStop = false;
				 this->groupBox1->Text = L"Camera";
				 // 
				 // Btn_CamCnt
				 // 
				 this->Btn_CamCnt->Location = System::Drawing::Point(6, 59);
				 this->Btn_CamCnt->Name = L"Btn_CamCnt";
				 this->Btn_CamCnt->Size = System::Drawing::Size(118, 66);
				 this->Btn_CamCnt->TabIndex = 13;
				 this->Btn_CamCnt->Text = L"開啟攝影機";
				 this->Btn_CamCnt->UseVisualStyleBackColor = true;
				 this->Btn_CamCnt->Click += gcnew System::EventHandler(this, &MyForm::Btn_CamCnt_Click);
				 // 
				 // cBox_CameraList
				 // 
				 this->cBox_CameraList->FormattingEnabled = true;
				 this->cBox_CameraList->Location = System::Drawing::Point(6, 21);
				 this->cBox_CameraList->Name = L"cBox_CameraList";
				 this->cBox_CameraList->Size = System::Drawing::Size(138, 20);
				 this->cBox_CameraList->TabIndex = 12;
				 // 
				 // groupBox5
				 // 
				 this->groupBox5->Controls->Add(this->label9);
				 this->groupBox5->Controls->Add(this->Btn_Radar_Connect);
				 this->groupBox5->Controls->Add(this->cBox_Radar);
				 this->groupBox5->Location = System::Drawing::Point(293, 18);
				 this->groupBox5->Name = L"groupBox5";
				 this->groupBox5->Size = System::Drawing::Size(200, 84);
				 this->groupBox5->TabIndex = 13;
				 this->groupBox5->TabStop = false;
				 this->groupBox5->Text = L"Radar";
				 // 
				 // label9
				 // 
				 this->label9->AutoSize = true;
				 this->label9->Location = System::Drawing::Point(18, 66);
				 this->label9->Name = L"label9";
				 this->label9->Size = System::Drawing::Size(41, 12);
				 this->label9->TabIndex = 14;
				 this->label9->Text = L"label9";
				 // 
				 // Btn_Radar_Connect
				 // 
				 this->Btn_Radar_Connect->Location = System::Drawing::Point(97, 19);
				 this->Btn_Radar_Connect->Name = L"Btn_Radar_Connect";
				 this->Btn_Radar_Connect->Size = System::Drawing::Size(75, 23);
				 this->Btn_Radar_Connect->TabIndex = 13;
				 this->Btn_Radar_Connect->Text = L"連接";
				 this->Btn_Radar_Connect->UseVisualStyleBackColor = true;
				 this->Btn_Radar_Connect->Click += gcnew System::EventHandler(this, &MyForm::Btn_Radar_Connect_Click);
				 // 
				 // cBox_Radar
				 // 
				 this->cBox_Radar->FormattingEnabled = true;
				 this->cBox_Radar->Location = System::Drawing::Point(6, 21);
				 this->cBox_Radar->Name = L"cBox_Radar";
				 this->cBox_Radar->Size = System::Drawing::Size(85, 20);
				 this->cBox_Radar->TabIndex = 12;
				 // 
				 // Btn_Refresh_Combox
				 // 
				 this->Btn_Refresh_Combox->Location = System::Drawing::Point(6, 577);
				 this->Btn_Refresh_Combox->Name = L"Btn_Refresh_Combox";
				 this->Btn_Refresh_Combox->Size = System::Drawing::Size(213, 106);
				 this->Btn_Refresh_Combox->TabIndex = 12;
				 this->Btn_Refresh_Combox->Text = L"Update Com";
				 this->Btn_Refresh_Combox->UseVisualStyleBackColor = true;
				 this->Btn_Refresh_Combox->Click += gcnew System::EventHandler(this, &MyForm::Btn_Refresh_Combox_Click);
				 // 
				 // groupBox9
				 // 
				 this->groupBox9->Controls->Add(this->Btn_UpDateSetting);
				 this->groupBox9->Controls->Add(this->label1);
				 this->groupBox9->Controls->Add(this->label21);
				 this->groupBox9->Controls->Add(this->cBox_LiDAR);
				 this->groupBox9->Controls->Add(this->label22);
				 this->groupBox9->Controls->Add(this->tBox_Partition);
				 this->groupBox9->Controls->Add(this->Btn_LiDARCnt);
				 this->groupBox9->Controls->Add(this->Btn_LiDARClose);
				 this->groupBox9->Location = System::Drawing::Point(6, 18);
				 this->groupBox9->Name = L"groupBox9";
				 this->groupBox9->Size = System::Drawing::Size(241, 161);
				 this->groupBox9->TabIndex = 10;
				 this->groupBox9->TabStop = false;
				 this->groupBox9->Text = L"LiDAR";
				 // 
				 // Btn_UpDateSetting
				 // 
				 this->Btn_UpDateSetting->Location = System::Drawing::Point(157, 120);
				 this->Btn_UpDateSetting->Name = L"Btn_UpDateSetting";
				 this->Btn_UpDateSetting->Size = System::Drawing::Size(75, 23);
				 this->Btn_UpDateSetting->TabIndex = 17;
				 this->Btn_UpDateSetting->Text = L"更新設定值";
				 this->Btn_UpDateSetting->UseVisualStyleBackColor = true;
				 this->Btn_UpDateSetting->Click += gcnew System::EventHandler(this, &MyForm::Btn_UpDateSetting_Click);
				 // 
				 // label1
				 // 
				 this->label1->AutoSize = true;
				 this->label1->Location = System::Drawing::Point(27, 22);
				 this->label1->Name = L"label1";
				 this->label1->Size = System::Drawing::Size(29, 12);
				 this->label1->TabIndex = 15;
				 this->label1->Text = L"Com:";
				 // 
				 // label21
				 // 
				 this->label21->AutoSize = true;
				 this->label21->Location = System::Drawing::Point(126, 123);
				 this->label21->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
				 this->label21->Name = L"label21";
				 this->label21->Size = System::Drawing::Size(23, 12);
				 this->label21->TabIndex = 14;
				 this->label21->Text = L"(m)";
				 // 
				 // cBox_LiDAR
				 // 
				 this->cBox_LiDAR->FormattingEnabled = true;
				 this->cBox_LiDAR->Location = System::Drawing::Point(62, 17);
				 this->cBox_LiDAR->Name = L"cBox_LiDAR";
				 this->cBox_LiDAR->Size = System::Drawing::Size(87, 20);
				 this->cBox_LiDAR->TabIndex = 0;
				 // 
				 // label22
				 // 
				 this->label22->AutoSize = true;
				 this->label22->Location = System::Drawing::Point(12, 123);
				 this->label22->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
				 this->label22->Name = L"label22";
				 this->label22->Size = System::Drawing::Size(59, 12);
				 this->label22->TabIndex = 13;
				 this->label22->Text = L"聚類閥值:";
				 // 
				 // tBox_Partition
				 // 
				 this->tBox_Partition->Location = System::Drawing::Point(77, 113);
				 this->tBox_Partition->Margin = System::Windows::Forms::Padding(2);
				 this->tBox_Partition->Name = L"tBox_Partition";
				 this->tBox_Partition->Size = System::Drawing::Size(36, 22);
				 this->tBox_Partition->TabIndex = 12;
				 this->tBox_Partition->Text = L"1";
				 // 
				 // Btn_LiDARCnt
				 // 
				 this->Btn_LiDARCnt->Location = System::Drawing::Point(157, 17);
				 this->Btn_LiDARCnt->Name = L"Btn_LiDARCnt";
				 this->Btn_LiDARCnt->Size = System::Drawing::Size(75, 23);
				 this->Btn_LiDARCnt->TabIndex = 5;
				 this->Btn_LiDARCnt->Text = L"連接";
				 this->Btn_LiDARCnt->UseVisualStyleBackColor = true;
				 this->Btn_LiDARCnt->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDARCnt_Click);
				 // 
				 // Btn_LiDARClose
				 // 
				 this->Btn_LiDARClose->Location = System::Drawing::Point(157, 57);
				 this->Btn_LiDARClose->Name = L"Btn_LiDARClose";
				 this->Btn_LiDARClose->Size = System::Drawing::Size(75, 23);
				 this->Btn_LiDARClose->TabIndex = 6;
				 this->Btn_LiDARClose->Text = L"關閉";
				 this->Btn_LiDARClose->UseVisualStyleBackColor = true;
				 this->Btn_LiDARClose->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDARClose_Click);
				 // 
				 // groupBox6
				 // 
				 this->groupBox6->Controls->Add(this->Btn_TboxClose);
				 this->groupBox6->Controls->Add(this->Tx_Radar_Mode);
				 this->groupBox6->Controls->Add(this->cBox_TBox);
				 this->groupBox6->Controls->Add(this->Btn_TboxCnt);
				 this->groupBox6->Location = System::Drawing::Point(12, 325);
				 this->groupBox6->Name = L"groupBox6";
				 this->groupBox6->Size = System::Drawing::Size(193, 246);
				 this->groupBox6->TabIndex = 9;
				 this->groupBox6->TabStop = false;
				 this->groupBox6->Text = L"TBox";
				 // 
				 // Btn_TboxClose
				 // 
				 this->Btn_TboxClose->Location = System::Drawing::Point(99, 54);
				 this->Btn_TboxClose->Margin = System::Windows::Forms::Padding(2);
				 this->Btn_TboxClose->Name = L"Btn_TboxClose";
				 this->Btn_TboxClose->Size = System::Drawing::Size(75, 23);
				 this->Btn_TboxClose->TabIndex = 12;
				 this->Btn_TboxClose->Text = L"關閉";
				 this->Btn_TboxClose->UseVisualStyleBackColor = true;
				 // 
				 // Tx_Radar_Mode
				 // 
				 this->Tx_Radar_Mode->AutoSize = true;
				 this->Tx_Radar_Mode->Location = System::Drawing::Point(6, 98);
				 this->Tx_Radar_Mode->Name = L"Tx_Radar_Mode";
				 this->Tx_Radar_Mode->Size = System::Drawing::Size(29, 12);
				 this->Tx_Radar_Mode->TabIndex = 11;
				 this->Tx_Radar_Mode->Text = L"Mode";
				 // 
				 // cBox_TBox
				 // 
				 this->cBox_TBox->FormattingEnabled = true;
				 this->cBox_TBox->Location = System::Drawing::Point(0, 26);
				 this->cBox_TBox->Name = L"cBox_TBox";
				 this->cBox_TBox->Size = System::Drawing::Size(87, 20);
				 this->cBox_TBox->TabIndex = 7;
				 // 
				 // Btn_TboxCnt
				 // 
				 this->Btn_TboxCnt->Location = System::Drawing::Point(99, 21);
				 this->Btn_TboxCnt->Name = L"Btn_TboxCnt";
				 this->Btn_TboxCnt->Size = System::Drawing::Size(75, 23);
				 this->Btn_TboxCnt->TabIndex = 8;
				 this->Btn_TboxCnt->Text = L"連接";
				 this->Btn_TboxCnt->UseVisualStyleBackColor = true;
				 this->Btn_TboxCnt->Click += gcnew System::EventHandler(this, &MyForm::Btn_TboxCnt_Click);
				 // 
				 // tabPage8
				 // 
				 this->tabPage8->Controls->Add(this->Tx_Result_Radar);
				 this->tabPage8->Controls->Add(this->Tx_Result_LIDAR);
				 this->tabPage8->Controls->Add(this->numericUpDown1);
				 this->tabPage8->Controls->Add(this->cBox_ShowResult);
				 this->tabPage8->Controls->Add(this->Tx_BarPos);
				 this->tabPage8->Controls->Add(this->cBox_ShowLiText);
				 this->tabPage8->Controls->Add(this->tx_TBox_RDataP3);
				 this->tabPage8->Controls->Add(this->tx_TBox_LDataP3);
				 this->tabPage8->Controls->Add(this->Tx_TBox_mode);
				 this->tabPage8->Controls->Add(this->label3);
				 this->tabPage8->Controls->Add(this->cBox_ScanPt);
				 this->tabPage8->Controls->Add(this->Btn_SpeedUp);
				 this->tabPage8->Controls->Add(this->Btn_SlowDown);
				 this->tabPage8->Controls->Add(this->Tx_MaxFrame);
				 this->tabPage8->Controls->Add(this->label2);
				 this->tabPage8->Controls->Add(this->trackBar1);
				 this->tabPage8->Controls->Add(this->tBox_LoadPath);
				 this->tabPage8->Controls->Add(this->Btn_LoadFilePath);
				 this->tabPage8->Controls->Add(this->Tx_CarSpeed2);
				 this->tabPage8->Controls->Add(this->Btn_PlayPause);
				 this->tabPage8->Controls->Add(this->pictureBox2);
				 this->tabPage8->Controls->Add(this->chart2);
				 this->tabPage8->Location = System::Drawing::Point(4, 22);
				 this->tabPage8->Name = L"tabPage8";
				 this->tabPage8->Size = System::Drawing::Size(1882, 974);
				 this->tabPage8->TabIndex = 2;
				 this->tabPage8->Text = L"回放";
				 this->tabPage8->UseVisualStyleBackColor = true;
				 // 
				 // Tx_Result_Radar
				 // 
				 this->Tx_Result_Radar->AutoSize = true;
				 this->Tx_Result_Radar->Location = System::Drawing::Point(1320, 878);
				 this->Tx_Result_Radar->Name = L"Tx_Result_Radar";
				 this->Tx_Result_Radar->Size = System::Drawing::Size(33, 12);
				 this->Tx_Result_Radar->TabIndex = 31;
				 this->Tx_Result_Radar->Text = L"label4";
				 // 
				 // Tx_Result_LIDAR
				 // 
				 this->Tx_Result_LIDAR->AutoSize = true;
				 this->Tx_Result_LIDAR->Location = System::Drawing::Point(1318, 850);
				 this->Tx_Result_LIDAR->Name = L"Tx_Result_LIDAR";
				 this->Tx_Result_LIDAR->Size = System::Drawing::Size(33, 12);
				 this->Tx_Result_LIDAR->TabIndex = 30;
				 this->Tx_Result_LIDAR->Text = L"label4";
				 // 
				 // numericUpDown1
				 // 
				 this->numericUpDown1->Location = System::Drawing::Point(1320, 802);
				 this->numericUpDown1->Name = L"numericUpDown1";
				 this->numericUpDown1->Size = System::Drawing::Size(120, 22);
				 this->numericUpDown1->TabIndex = 29;
				 this->numericUpDown1->Visible = false;
				 this->numericUpDown1->ValueChanged += gcnew System::EventHandler(this, &MyForm::numericUpDown1_ValueChanged);
				 // 
				 // cBox_ShowResult
				 // 
				 this->cBox_ShowResult->AutoSize = true;
				 this->cBox_ShowResult->Location = System::Drawing::Point(1555, 761);
				 this->cBox_ShowResult->Name = L"cBox_ShowResult";
				 this->cBox_ShowResult->Size = System::Drawing::Size(79, 16);
				 this->cBox_ShowResult->TabIndex = 28;
				 this->cBox_ShowResult->Text = L"ShowResult";
				 this->cBox_ShowResult->UseVisualStyleBackColor = true;
				 this->cBox_ShowResult->Visible = false;
				 this->cBox_ShowResult->CheckedChanged += gcnew System::EventHandler(this, &MyForm::cBox_ShowResult_CheckedChanged);
				 // 
				 // Tx_BarPos
				 // 
				 this->Tx_BarPos->AutoSize = true;
				 this->Tx_BarPos->Location = System::Drawing::Point(1548, 719);
				 this->Tx_BarPos->Name = L"Tx_BarPos";
				 this->Tx_BarPos->Size = System::Drawing::Size(33, 12);
				 this->Tx_BarPos->TabIndex = 27;
				 this->Tx_BarPos->Text = L"label4";
				 // 
				 // cBox_ShowLiText
				 // 
				 this->cBox_ShowLiText->AutoSize = true;
				 this->cBox_ShowLiText->Checked = true;
				 this->cBox_ShowLiText->CheckState = System::Windows::Forms::CheckState::Checked;
				 this->cBox_ShowLiText->Location = System::Drawing::Point(1449, 761);
				 this->cBox_ShowLiText->Name = L"cBox_ShowLiText";
				 this->cBox_ShowLiText->Size = System::Drawing::Size(100, 16);
				 this->cBox_ShowLiText->TabIndex = 26;
				 this->cBox_ShowLiText->Text = L"Show Obj Value";
				 this->cBox_ShowLiText->UseVisualStyleBackColor = true;
				 // 
				 // tx_TBox_RDataP3
				 // 
				 this->tx_TBox_RDataP3->AutoSize = true;
				 this->tx_TBox_RDataP3->Location = System::Drawing::Point(1637, 411);
				 this->tx_TBox_RDataP3->Name = L"tx_TBox_RDataP3";
				 this->tx_TBox_RDataP3->Size = System::Drawing::Size(33, 12);
				 this->tx_TBox_RDataP3->TabIndex = 25;
				 this->tx_TBox_RDataP3->Text = L"label4";
				 // 
				 // tx_TBox_LDataP3
				 // 
				 this->tx_TBox_LDataP3->AutoSize = true;
				 this->tx_TBox_LDataP3->Location = System::Drawing::Point(1407, 411);
				 this->tx_TBox_LDataP3->Name = L"tx_TBox_LDataP3";
				 this->tx_TBox_LDataP3->Size = System::Drawing::Size(33, 12);
				 this->tx_TBox_LDataP3->TabIndex = 25;
				 this->tx_TBox_LDataP3->Text = L"label4";
				 // 
				 // Tx_TBox_mode
				 // 
				 this->Tx_TBox_mode->AutoSize = true;
				 this->Tx_TBox_mode->Location = System::Drawing::Point(1336, 478);
				 this->Tx_TBox_mode->Name = L"Tx_TBox_mode";
				 this->Tx_TBox_mode->Size = System::Drawing::Size(33, 12);
				 this->Tx_TBox_mode->TabIndex = 24;
				 this->Tx_TBox_mode->Text = L"label4";
				 // 
				 // label3
				 // 
				 this->label3->AutoSize = true;
				 this->label3->Location = System::Drawing::Point(1332, 448);
				 this->label3->Name = L"label3";
				 this->label3->Size = System::Drawing::Size(36, 12);
				 this->label3->TabIndex = 23;
				 this->label3->Text = L"Speed:";
				 // 
				 // cBox_ScanPt
				 // 
				 this->cBox_ScanPt->AutoSize = true;
				 this->cBox_ScanPt->Checked = true;
				 this->cBox_ScanPt->CheckState = System::Windows::Forms::CheckState::Checked;
				 this->cBox_ScanPt->Location = System::Drawing::Point(1327, 761);
				 this->cBox_ScanPt->Name = L"cBox_ScanPt";
				 this->cBox_ScanPt->Size = System::Drawing::Size(84, 16);
				 this->cBox_ScanPt->TabIndex = 22;
				 this->cBox_ScanPt->Text = L"光達掃描點";
				 this->cBox_ScanPt->UseVisualStyleBackColor = true;
				 // 
				 // Btn_SpeedUp
				 // 
				 this->Btn_SpeedUp->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"Btn_SpeedUp.Image")));
				 this->Btn_SpeedUp->Location = System::Drawing::Point(1639, 572);
				 this->Btn_SpeedUp->Name = L"Btn_SpeedUp";
				 this->Btn_SpeedUp->Size = System::Drawing::Size(74, 69);
				 this->Btn_SpeedUp->TabIndex = 20;
				 this->Btn_SpeedUp->UseVisualStyleBackColor = true;
				 this->Btn_SpeedUp->Click += gcnew System::EventHandler(this, &MyForm::Btn_SpeedUp_Click);
				 // 
				 // Btn_SlowDown
				 // 
				 this->Btn_SlowDown->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"Btn_SlowDown.Image")));
				 this->Btn_SlowDown->Location = System::Drawing::Point(1423, 572);
				 this->Btn_SlowDown->Name = L"Btn_SlowDown";
				 this->Btn_SlowDown->Size = System::Drawing::Size(76, 69);
				 this->Btn_SlowDown->TabIndex = 20;
				 this->Btn_SlowDown->UseVisualStyleBackColor = true;
				 this->Btn_SlowDown->Click += gcnew System::EventHandler(this, &MyForm::Btn_SlowDown_Click);
				 // 
				 // Tx_MaxFrame
				 // 
				 this->Tx_MaxFrame->AutoSize = true;
				 this->Tx_MaxFrame->Location = System::Drawing::Point(1797, 719);
				 this->Tx_MaxFrame->Name = L"Tx_MaxFrame";
				 this->Tx_MaxFrame->Size = System::Drawing::Size(33, 12);
				 this->Tx_MaxFrame->TabIndex = 19;
				 this->Tx_MaxFrame->Text = L"label3";
				 // 
				 // label2
				 // 
				 this->label2->AutoSize = true;
				 this->label2->Location = System::Drawing::Point(1336, 719);
				 this->label2->Name = L"label2";
				 this->label2->Size = System::Drawing::Size(11, 12);
				 this->label2->TabIndex = 18;
				 this->label2->Text = L"0";
				 // 
				 // trackBar1
				 // 
				 this->trackBar1->BackColor = System::Drawing::SystemColors::ControlLightLight;
				 this->trackBar1->Location = System::Drawing::Point(1327, 686);
				 this->trackBar1->Name = L"trackBar1";
				 this->trackBar1->Size = System::Drawing::Size(494, 45);
				 this->trackBar1->TabIndex = 17;
				 this->trackBar1->Scroll += gcnew System::EventHandler(this, &MyForm::trackBar1_Scroll);
				 // 
				 // tBox_LoadPath
				 // 
				 this->tBox_LoadPath->Location = System::Drawing::Point(1334, 515);
				 this->tBox_LoadPath->Name = L"tBox_LoadPath";
				 this->tBox_LoadPath->Size = System::Drawing::Size(412, 22);
				 this->tBox_LoadPath->TabIndex = 16;
				 // 
				 // Btn_LoadFilePath
				 // 
				 this->Btn_LoadFilePath->Location = System::Drawing::Point(1752, 504);
				 this->Btn_LoadFilePath->Name = L"Btn_LoadFilePath";
				 this->Btn_LoadFilePath->Size = System::Drawing::Size(69, 41);
				 this->Btn_LoadFilePath->TabIndex = 15;
				 this->Btn_LoadFilePath->Text = L"...";
				 this->Btn_LoadFilePath->UseVisualStyleBackColor = true;
				 this->Btn_LoadFilePath->Click += gcnew System::EventHandler(this, &MyForm::Btn_LoadFilePath_Click);
				 // 
				 // Tx_CarSpeed2
				 // 
				 this->Tx_CarSpeed2->AutoSize = true;
				 this->Tx_CarSpeed2->Location = System::Drawing::Point(1374, 448);
				 this->Tx_CarSpeed2->Name = L"Tx_CarSpeed2";
				 this->Tx_CarSpeed2->Size = System::Drawing::Size(33, 12);
				 this->Tx_CarSpeed2->TabIndex = 14;
				 this->Tx_CarSpeed2->Text = L"label1";
				 // 
				 // Btn_PlayPause
				 // 
				 this->Btn_PlayPause->Enabled = false;
				 this->Btn_PlayPause->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"Btn_PlayPause.Image")));
				 this->Btn_PlayPause->Location = System::Drawing::Point(1537, 572);
				 this->Btn_PlayPause->Name = L"Btn_PlayPause";
				 this->Btn_PlayPause->Size = System::Drawing::Size(69, 69);
				 this->Btn_PlayPause->TabIndex = 13;
				 this->Btn_PlayPause->UseVisualStyleBackColor = true;
				 this->Btn_PlayPause->Click += gcnew System::EventHandler(this, &MyForm::Btn_PlayPause_Click);
				 // 
				 // pictureBox2
				 // 
				 this->pictureBox2->Location = System::Drawing::Point(1334, 3);
				 this->pictureBox2->Name = L"pictureBox2";
				 this->pictureBox2->Size = System::Drawing::Size(487, 384);
				 this->pictureBox2->TabIndex = 12;
				 this->pictureBox2->TabStop = false;
				 // 
				 // chart2
				 // 
				 chartArea4->AxisX->Interval = 100;
				 chartArea4->AxisX->IsReversed = true;
				 chartArea4->AxisX->Maximum = 1000;
				 chartArea4->AxisX->Minimum = -1000;
				 chartArea4->AxisY->Interval = 100;
				 chartArea4->AxisY->IsReversed = true;
				 chartArea4->AxisY->Maximum = 4000;
				 chartArea4->AxisY->Minimum = 0;
				 chartArea4->Name = L"ChartArea1";
				 this->chart2->ChartAreas->Add(chartArea4);
				 legend4->Name = L"Legend1";
				 this->chart2->Legends->Add(legend4);
				 this->chart2->Location = System::Drawing::Point(-42, -32);
				 this->chart2->Name = L"chart2";
				 series17->ChartArea = L"ChartArea1";
				 series17->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series17->Color = System::Drawing::Color::Khaki;
				 series17->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 series17->LabelForeColor = System::Drawing::Color::YellowGreen;
				 series17->Legend = L"Legend1";
				 series17->MarkerSize = 3;
				 series17->Name = L"Series_LiDAR";
				 series18->ChartArea = L"ChartArea1";
				 series18->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series18->Color = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)),
					 static_cast<System::Int32>(static_cast<System::Byte>(192)));
				 series18->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
					 static_cast<System::Byte>(0)));
				 series18->Legend = L"Legend1";
				 series18->MarkerColor = System::Drawing::Color::Blue;
				 series18->MarkerSize = 10;
				 series18->Name = L"Series_LiDAR_CLOSE";
				 series19->ChartArea = L"ChartArea1";
				 series19->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::FastPoint;
				 series19->Legend = L"Legend1";
				 series19->MarkerColor = System::Drawing::Color::Red;
				 series19->MarkerSize = 15;
				 series19->Name = L"Series_TBox_Radar";
				 series20->ChartArea = L"ChartArea1";
				 series20->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series20->Legend = L"Legend1";
				 series20->MarkerSize = 10;
				 series20->Name = L"Series_RadarDetectArea";
				 this->chart2->Series->Add(series17);
				 this->chart2->Series->Add(series18);
				 this->chart2->Series->Add(series19);
				 this->chart2->Series->Add(series20);
				 this->chart2->Size = System::Drawing::Size(1389, 977);
				 this->chart2->TabIndex = 11;
				 this->chart2->Text = L"圖";
				 // 
				 // serialPort_LiDAR
				 // 
				 this->serialPort_LiDAR->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_LiDAR_DataReceived);
				 // 
				 // timer1
				 // 
				 this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
				 // 
				 // serialPort_Tbox
				 // 
				 this->serialPort_Tbox->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Tbox_DataReceived);
				 // 
				 // timer2
				 // 
				 this->timer2->Tick += gcnew System::EventHandler(this, &MyForm::timer2_Tick);
				 // 
				 // MyForm
				 // 
				 this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
				 this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
				 this->AutoSizeMode = System::Windows::Forms::AutoSizeMode::GrowAndShrink;
				 this->ClientSize = System::Drawing::Size(1874, 961);
				 this->Controls->Add(this->tabControl1);
				 this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedSingle;
				 this->Name = L"MyForm";
				 this->StartPosition = System::Windows::Forms::FormStartPosition::CenterScreen;
				 this->Text = L"MyForm";
				 this->tabControl1->ResumeLayout(false);
				 this->tabPage1->ResumeLayout(false);
				 this->tabPage1->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
				 this->tabPage2->ResumeLayout(false);
				 this->groupBox1->ResumeLayout(false);
				 this->groupBox5->ResumeLayout(false);
				 this->groupBox5->PerformLayout();
				 this->groupBox9->ResumeLayout(false);
				 this->groupBox9->PerformLayout();
				 this->groupBox6->ResumeLayout(false);
				 this->groupBox6->PerformLayout();
				 this->tabPage8->ResumeLayout(false);
				 this->tabPage8->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown1))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->EndInit();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart2))->EndInit();
				 this->ResumeLayout(false);

			 }
#pragma endregion
	private:System::String^ getRadarMode(int index)
	{
		switch (index)
		{
		case 0x01:
			return "BSD Mode";
			break;
		case 0x02:
			return  "RCTA Mode";
			break;
		case 0x03:
			return  "DOW Mode";
			break;
		}
	}
	private: System::Void Btn_Refresh_Combox_Click(System::Object^  sender, System::EventArgs^  e) {
		ComPortRefresh();
	}
	private:void ComPortRefresh(void)
	{
		cBox_LiDAR->Items->Clear();
		cBox_TBox->Items->Clear();
		cBox_Radar->Items->Clear();
		cBox_CameraList->Items->Clear();

		cli::array<System::String^>^ Port = SerialPort::GetPortNames();
		cBox_LiDAR->Items->AddRange(Port);
		cBox_TBox->Items->AddRange(Port);
		cBox_Radar->Items->AddRange(Port);
		std::map<int, Device> devices = de.getVideoDevicesMap();
		devices = de.getVideoDevicesMap();
		for (uint i = 0; i < devices.size(); i++)
		{
			System::String ^str = gcnew System::String(devices[i].deviceName.c_str());
			str = devices[i].id.ToString() + ":" + str;
			cBox_CameraList->Items->Add(str);
		}
	}
	private: System::Void Btn_CamCnt_Click(System::Object^  sender, System::EventArgs^  e) {
		if (cap.isOpened())cap.release();
		//cap.open(CV_CAP_DSHOW);
		cap.open(Convert::ToInt16(cBox_CameraList->Text->Substring(0, 1)));
		ComPortNoRecord[2] = Convert::ToInt16(cBox_CameraList->Text->Substring(0, 1));
		string str = FileNameTime + (string)"\\VideoTest.avi";
		videoWrite.open(str, CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640, 480));

		fstream fp_ComID;
		fp_ComID.open("ComRecord.txt", ios::out);
		fp_ComID << ComPortNoRecord[0] << " " << ComPortNoRecord[1] << " " << ComPortNoRecord[2] << endl;
		fp_ComID.close();
	}
	private: System::Void serialPort_LiDAR_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^ LiDAR_SerialPortData = gcnew cli::array<Byte>(10000);
		int ReadSize = serialPort_LiDAR->Read(LiDAR_SerialPortData, 0, 10000);

		for (int i = 0; i < ReadSize; i++)
		{
			if ((LiDAR_SerialPortData[i] == 0x02) && (LiDAR_SerialPortData[i + 1] == 0x80) && (LiDAR_SerialPortData[i + 4] == 0xB0) && (f_getHeader == false))
			{

				for (int j = i; j < ReadSize; j++)
				{
					LiDAR_Data[counter] = LiDAR_SerialPortData[j];
					counter++;
					if ((counter + 1) == 734)
						break;
				}
				f_getHeader = true;
				break;
			}
			if (f_getHeader)
			{
				if ((counter + 1) == 734)
					break;
				LiDAR_Data[counter] = LiDAR_SerialPortData[i];
				counter++;
			}
		}
		int Data[361];
		if ((counter + 1) == 734)
		{
			for (uint i = 0; i < 361; i++)
			{
				Data[i] = LiDAR_Data[2 * i + 7] + (LiDAR_Data[8 + i * 2] & 0x1F) * 256;
			}
			for (uint i = 0; i < 361; i++)
			{
				LIDAR_R_cooridate[i] = Data[i];
				LIDAR_X_cooridate[i] = (Data[i]) * cos((0.5 * i) * (M_PI / 180));
				LIDAR_Y_cooridate[i] = (Data[i]) * sin((0.5 * i) * (M_PI / 180));
			}
			f_getLiDARData = true;
			f_getHeader = false;
			counter = 0;
		}
	}
	private: System::Void Btn_LiDARCnt_Click(System::Object^  sender, System::EventArgs^  e) {
		if (serialPort_LiDAR->IsOpen)serialPort_LiDAR->Close();
		serialPort_LiDAR->PortName = cBox_LiDAR->Text;
		serialPort_LiDAR->Encoding = System::Text::Encoding::GetEncoding(28591);
		serialPort_LiDAR->BaudRate = 9600;
		serialPort_LiDAR->DataBits = 8;
		serialPort_LiDAR->StopBits = StopBits::One;
		serialPort_LiDAR->Parity = Parity::None;
		serialPort_LiDAR->Open();

		cli::array<System::Byte>^ LMS_Angular_range_change_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x05, 0x00, 0x3B, 0xB4, 0x00, 0x32, 0x00, 0x3B, 0x1F };//更改LMS經度0.5度
		cli::array<System::Byte>^ continuous_LMS_data_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08 };//更改成連續指令緩區
		cli::array<System::Byte>^ LMS_baundrate_500k_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x48, 0x58, 0x08 };//更改包率

		if (serialPort_LiDAR->IsOpen)
		{
			serialPort_LiDAR->Write(LMS_baundrate_500k_manage, 0, 8);
			Sleep(500);
			serialPort_LiDAR->Close();
			serialPort_LiDAR->BaudRate = 500000;
			serialPort_LiDAR->Open();
		}
		if (serialPort_LiDAR->IsOpen)
		{
			serialPort_LiDAR->Write(LMS_Angular_range_change_manage, 0, 11);
			_sleep(500);
			serialPort_LiDAR->Write(continuous_LMS_data_manage, 0, 8);
		}

		ComPortNoRecord[0] = Convert::ToInt16(cBox_LiDAR->Text->Remove(0, 3));

		fstream fp_ComID;
		fp_ComID.open("ComRecord.txt", ios::out);
		fp_ComID << ComPortNoRecord[0] << " " << ComPortNoRecord[1] << " " << ComPortNoRecord[2] << endl;
		fp_ComID.close();
	}
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
		if (cap.isOpened())
		{
			Mat frame1;
			cap >> frame1;
			videoWrite.write(frame1);
			videoWrite.set(CV_CAP_PROP_POS_MSEC, t1);
			ShowImage(pictureBox1, frame1);
		}
		chart1->Series["Series_LiDAR"]->Points->Clear();
		chart1->Series["Series_LiDAR_CLOSE"]->Points->Clear();
		chart1->Series["Series_Radar_Angle"]->Points->Clear();
		vector<Pt> Pt_newClusterRefPt;
		/*	if (!cBox_Record->Checked)
			{*/
		chart1->Series["Series_TBox_RRadar"]->Points->Clear();
		chart1->Series["Series_TBox_LRadar"]->Points->Clear();
		//}
#pragma region 光達
		if (serialPort_LiDAR->IsOpen)
		{

			vector<Pt>LIDAR_cooridate;
			LIDAR_cooridate.resize(0);
			LIDAR_cooridate.resize(361);
			fstream fp_Lidar; fp_Lidar.open(".\\" + FileNameTime + "\\Lidar.txt", ios::out | ios::app);

			for (uint i = 0; i < 361; i++)
			{
				LIDAR_cooridate[i] = Pt(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i]);
				LIDAR_cooridate[i].range = LIDAR_R_cooridate[i];
				fp_Lidar << LIDAR_R_cooridate[i] << " ";
			}
			fp_Lidar << t1 << " " << TBox.currentSpeed << endl;
			fp_Lidar.close();
			vector<int >lab;

			//int nObj = EuclidCluster(LIDAR_cooridate, PartitionValue);
			//vector<vector<Pt>> Pt_ClusterList_new = Cluster2List(LIDAR_cooridate, nObj);
			vector<vector<Pt>> Pt_ClusterList_new = Cluster2List_ContinuousAngle(LIDAR_cooridate);
			Pt_newClusterRefPt.resize(Pt_ClusterList_new.size());
			int index = 0;
			for (uint16_t i = 0; i < Pt_ClusterList_new.size(); i++)
			{
				double  minX = 8000;
				double minY = 8000;
				Pt min;
				Color color = Color::FromArgb(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				for (uint j = 0; j < Pt_ClusterList_new[i].size(); j++)
				{
					if (abs(Pt_ClusterList_new[i][j].x) < minX)
					{
						min.x = Pt_ClusterList_new[i][j].x;
						minX = abs(Pt_ClusterList_new[i][j].x);
					}
					if (abs(Pt_ClusterList_new[i][j].y) < minY)
					{
						minY = abs(Pt_ClusterList_new[i][j].y);
						min.y = Pt_ClusterList_new[i][j].y;
					}
					chart1->Series["Series_LiDAR"]->Points->AddXY(Pt_ClusterList_new[i][j].x, Pt_ClusterList_new[i][j].y);
					chart1->Series["Series_LiDAR"]->Points[index]->Color = color;
					index++;
				}
				Pt_newClusterRefPt[i] = min;
				Pt_newClusterRefPt[i].range = get_Distance(min, Pt(0, 0));
			}
			if (Pt_oldClusterRefPoint.size() == 0) {
				Pt_oldClusterRefPoint = Pt_newClusterRefPt;
			}
			time_t t2 = clock();
			float time = (float)(t2 - t1) / CLK_TCK;
			t1 = t2;
			FindClosePoint(Pt_newClusterRefPt, Pt_oldClusterRefPoint, time, CurrentSpeed);
			chart1->Refresh();
			Pt_oldClusterRefPoint = Pt_newClusterRefPt;
		}
#pragma endregion



#pragma region TBox
		if (serialPort_Tbox->IsOpen)
		{

			if (std::abs(f_model_changed - TBox.L_RADAR_Mode) > 0)
			{
				//
				chart1->Series["Series_RadarDetectArea"]->Points->Clear();
				DrawBoundary(chart1, TBox.L_RADAR_Mode);
				f_model_changed = TBox.L_RADAR_Mode;
				Tx_Radar_Mode->Text = "L:" + getRadarMode(TBox.L_RADAR_Mode) + "  R:" + getRadarMode(TBox.R_RADAR_Mode);
			}

			if (TBox.Passenger_RADAR_ALert)
			{
				Pt R_RadarPtAtLiDAR = R_Radar2LiDAR(Pt(100 * TBox.R_RADAR_Range*Math::Cos(TBox.R_RADAR_Angle*M_PI / 180.f), 100 * TBox.R_RADAR_Range*Math::Sin(TBox.R_RADAR_Angle*M_PI / 180.f)));
				tx_TBox_RAngle->ForeColor = Color::Red;
				tx_TBox_RAngle->Text = "R Range: " + TBox.R_RADAR_Range.ToString() + " R Angle: " + TBox.R_RADAR_Angle.ToString() + "R Speed: " + TBox.R_RADAR_Speed.ToString();

				chart1->Series["Series_TBox_LRadar"]->Points->AddXY(R_RadarPtAtLiDAR.x, R_RadarPtAtLiDAR.y);
			}
			else
				tx_TBox_RAngle->ForeColor = Color::Blue;

			if (TBox.Driver_RADAR_ALert)
			{

				tx_TBox_LAngle->ForeColor = Color::Red;
				tx_TBox_LAngle->Text = "L Range: " + TBox.L_RADAR_Range.ToString() + " L Angle: " + TBox.L_RADAR_Angle.ToString() + " L Speed: " + TBox.L_RADAR_Speed.ToString();
				Pt L_RadarPtAtLiDAR = L_Radar2LiDAR(Pt(100 * TBox.L_RADAR_Range*Math::Cos(TBox.L_RADAR_Angle*M_PI / 180.f), 100 * TBox.L_RADAR_Range*Math::Sin(TBox.L_RADAR_Angle*M_PI / 180.f)));
				chart1->Series["Series_TBox_LRadar"]->Points->AddXY(L_RadarPtAtLiDAR.x, L_RadarPtAtLiDAR.y);
			}
			else
				tx_TBox_LAngle->ForeColor = Color::Blue;
			Tx_CarSpeed->Text = TBox.currentSpeed.ToString();

			fstream fp_TBox;
			fp_TBox.open(".\\" + FileNameTime + "\\TBox.txt", ios::out | ios::app);
			fp_TBox << TBox.currentSpeed << " "
				<< TBox.L_RADAR_Mode << " "
				<< TBox.Driver_RADAR_ALert << " "
				<< TBox.L_RADAR_Range << " "
				<< TBox.L_RADAR_Speed << " "
				<< TBox.L_RADAR_Angle << " "
				<< TBox.R_RADAR_Mode << " "
				<< TBox.Passenger_RADAR_ALert << " "
				<< TBox.R_RADAR_Range << " "
				<< TBox.R_RADAR_Speed << " "
				<< TBox.R_RADAR_Angle << " "
				<< System::DateTime::Now.Minute << " " << System::DateTime::Now.Second << endl;
			fp_TBox.close();
			tx_TBox_RAngle->Text = "R Range: " + TBox.R_RADAR_Range.ToString() + " R Angle: " + TBox.R_RADAR_Angle.ToString() + " R Speed: " + TBox.R_RADAR_Speed.ToString();


		}
#pragma endregion

#pragma region DB9雷達
		if (serialPort_Radar->IsOpen)
		{
			if (RadarData.ALert > 0)
			{
				fstream fptemp;
				char RadarFileName[100];
				int Date = System::DateTime::Now.Day * 10000 + System::DateTime::Now.Hour * 100 + System::DateTime::Now.Minute;
				sprintf(RadarFileName, "RadarData%d.txt", Date);
				fptemp.open(RadarFileName, ios::out | ios::app);
				fptemp << RadarData.Range << "\t" << RadarData.Angle << "\t" << RadarData.Speed << endl;
				fptemp.close();
				label9->Text = RadarData.Range.ToString() + " " + RadarData.Angle.ToString();
			}
		}
#pragma endregion	
		chart1->Refresh();
	}
	public:void ShowImage(System::Windows::Forms::PictureBox^ PBox, cv::Mat Image)
	{
		Mat image_Temp;
		switch (Image.type())
		{
		case CV_8UC3:
			Image.copyTo(image_Temp);

			break;
		case CV_8UC1:
			cvtColor(Image, image_Temp, CV_GRAY2RGB);
			break;
		default:
			break;
		}
		if (image_Temp.empty())return;
		Bitmap ^ bmpimg = gcnew Bitmap(image_Temp.cols, image_Temp.rows, System::Drawing::Imaging::PixelFormat::Format24bppRgb);
		System::Drawing::Imaging::BitmapData^ data = bmpimg->LockBits(System::Drawing::Rectangle(0, 0, image_Temp.cols, image_Temp.rows), System::Drawing::Imaging::ImageLockMode::WriteOnly, System::Drawing::Imaging::PixelFormat::Format24bppRgb);
		Byte* dstData = reinterpret_cast<Byte*>(data->Scan0.ToPointer());

		unsigned char* srcData = image_Temp.data;

		for (int row = 0; row < data->Height; ++row)
		{
			memcpy(reinterpret_cast<void*>(&dstData[row*data->Stride]), reinterpret_cast<void*>(&srcData[row*image_Temp.step]), image_Temp.cols*image_Temp.channels());
		}

		bmpimg->UnlockBits(data);
		PBox->Image = bmpimg;
		PBox->SizeMode = PictureBoxSizeMode::StretchImage;
		PBox->Refresh();
		GC::Collect();
	}
	private: System::Void btn_RecordCnt_Click(System::Object^  sender, System::EventArgs^  e) {


		Btn_LiDARCnt->PerformClick();
		_sleep(500);
		Btn_TboxCnt->PerformClick();
		Btn_CamCnt->PerformClick();
	}

	private: System::Void Btn_LiDARClose_Click(System::Object^  sender, System::EventArgs^  e) {
		cli::array<System::Byte>^ LMS_Stope_manage = gcnew cli::array<System::Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x38, 0x08};
		serialPort_LiDAR->Write(LMS_Stope_manage, 0, 8);
		serialPort_LiDAR->Close();
	}
	private:void DrawBoundary(System::Windows::Forms::DataVisualization::Charting::Chart ^% _chart, uint index)
	{
		switch (index)
		{
		case 1://BSD
		{
			_chart->ChartAreas[0]->AxisX->Maximum = 1000;
			_chart->ChartAreas[0]->AxisX->Minimum = -1000;

			for (uint j = 400; j <= 1200; j += 100)
			{
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + 380, left_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - 380, right_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x, left_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x, right_Radar_bias.y + j);

			}
			for (int i = 0; i < 400; i += 100)
			{
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + i, left_Radar_bias.y + 400);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - i, right_Radar_bias.y + 400);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + i, left_Radar_bias.y + 1200);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - i, right_Radar_bias.y + 1200);

			}

		}
		break;
		case 0x02://RCTA
			_chart->ChartAreas[0]->AxisX->Maximum = 2500;
			_chart->ChartAreas[0]->AxisX->Minimum = -2500;

			for (uint j = 0; j <= 1000; j += 100)
			{
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - 2000, right_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + 2000, left_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x, right_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x, left_Radar_bias.y + j);
			}

			for (int i = 0; i <= 2000; i += 100)
			{
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - i, 0);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + i, 0);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - i, 1000 + right_Radar_bias.y);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + i, 1000 + left_Radar_bias.y);
			}

			break;
		case 0x03://DOW
			_chart->ChartAreas[0]->AxisX->Maximum = 1000;
			_chart->ChartAreas[0]->AxisX->Minimum = -1000;
			for (uint j = 0; j <= 2000; j += 100)
			{
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + 380, left_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - 380, right_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x, left_Radar_bias.y + j);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x, right_Radar_bias.y + j);
			}

			for (int i = 0; i < 380; i += 100)
			{
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + i, 0);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - i, 0);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(left_Radar_bias.x + i, left_Radar_bias.y + 2000);
				_chart->Series["Series_RadarDetectArea"]->Points->AddXY(right_Radar_bias.x - i, right_Radar_bias.y + 2000);
			}
			break;
		default:
			break;
		}

	}

	private: System::Void Btn_Radar_Connect_Click(System::Object^  sender, System::EventArgs^  e) {
		serialPort_Radar->PortName = cBox_Radar->Text;
		serialPort_Radar->BaudRate = 460800;
		serialPort_Radar->DataBits = 8;
		serialPort_Radar->StopBits = StopBits::One;
		serialPort_Radar->Parity = Parity::None;
		serialPort_Radar->Open();
	}
	private: System::Void Btn_TboxCnt_Click(System::Object^  sender, System::EventArgs^  e) {
		if (serialPort_Tbox->IsOpen)
		{
			serialPort_Tbox->Close();
			Sleep(10);
		}
		serialPort_Tbox->PortName = cBox_TBox->Text;
		serialPort_Tbox->BaudRate = 115200;
		serialPort_Tbox->DataBits = 8;
		serialPort_Tbox->StopBits = StopBits::One;
		serialPort_Tbox->Parity = Parity::None;
		serialPort_Tbox->Open();

		ComPortNoRecord[1] = Convert::ToInt16(cBox_TBox->Text->Remove(0, 3));

		fstream fp_ComID;
		fp_ComID.open("ComRecord.txt", ios::out);
		fp_ComID << ComPortNoRecord[0] << " " << ComPortNoRecord[1] << " " << ComPortNoRecord[2] << endl;
		fp_ComID.close();
	}

	private:bool _is_InRange(Pt P)
	{
		switch (TBox.L_RADAR_Mode)
		{
		case 0x01:
			if (abs(P.x) < 500 && P.y < 1200)//BSD
				return true;
			else
				return false;

			break;
		case 0x02:
			if (abs(P.x) < 2200 && P.y < 1000)//RCTA
				return true;
			else
				return false;
			break;
		case 0x03:
			if (abs(P.x) < 500 && P.y < 2000)//DOW
				return true;
			else
				return false;
			break;
		}
	}

	private: System::Void serialPort_Tbox_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^bTboxData = gcnew cli::array<Byte>(122);

		byte Checksum = 0;
		int ReadLength = serialPort_Tbox->Read(bTboxData, 0, 122);
		for (uint i = 0; i < ReadLength; i++)
		{
			if (bTboxData[i] == 0x86)
			{
				for (uint j = i; j < i + 24; j++)
				{
					Checksum += bTboxData[j];
				}

				if (Checksum == bTboxData[i + format - 1])
				{
					TBox.currentSpeed = bTboxData[i + 9];
					TBox.L_RADAR_Mode = bTboxData[i + 14];
					TBox.Driver_RADAR_ALert = bTboxData[i + 15];
					TBox.L_RADAR_Range = bTboxData[i + 16];
					TBox.L_RADAR_Speed = bTboxData[i + 17] - 127;
					TBox.L_RADAR_Angle = bTboxData[i + 18] - 127;
					TBox.R_RADAR_Mode = bTboxData[i + 19];
					TBox.Passenger_RADAR_ALert = bTboxData[i + 20];
					TBox.R_RADAR_Range = bTboxData[i + 21];
					TBox.R_RADAR_Speed = bTboxData[i + 22] - 127;
					TBox.R_RADAR_Angle = bTboxData[i + 23] - 127;
					CurrentSpeed = bTboxData[i + 9];
				}

			}

		}


		if (serialPort_Tbox->BytesToRead >= 122)
		{
			serialPort_Tbox->DiscardInBuffer();
		}
	}


	private:void LoadData()
	{
		std::fstream fp;
		fp.open("LBias.txt", std::ios::in);
		fp >> left_Radar_bias.x;
		fp >> left_Radar_bias.y;
		fp.close();
		fp.open("RBias.txt", std::ios::in);
		fp >> right_Radar_bias.x;
		fp >> right_Radar_bias.y;
		fp.close();

	}
	private: System::Void Btn_UpDateSetting_Click(System::Object^  sender, System::EventArgs^  e) {

		PartitionValue = Convert::ToDouble(tBox_Partition->Text) * 100;
	}
	private:Pt R_Radar2LiDAR(Pt P)
	{
		Pt Ans;
		Pt Rotation = CoordinateRotation(-125, P);
		Ans.x = Rotation.x + right_Radar_bias.x;
		Ans.y = Rotation.y + right_Radar_bias.y;
		return Ans;
	}
	private:Pt L_Radar2LiDAR(Pt P)
	{
		Pt Ans;
		Pt Rotationtmp = CoordinateRotation(35.0f, P);
		Ans.x = Rotationtmp.y + left_Radar_bias.x;
		Ans.y = Rotationtmp.x + left_Radar_bias.y;
		return Ans;
	}
	private:void LoadComPort()
	{
		fstream fp_ComID;
		fp_ComID.open("ComRecord.txt", ios::in);
		fp_ComID >> ComPortNoRecord[0]; fp_ComID >> ComPortNoRecord[1]; fp_ComID >> ComPortNoRecord[2];
		fp_ComID.close();
		cBox_LiDAR->Text = "COM" + ComPortNoRecord[0].ToString();
		cBox_TBox->Text = "COM" + ComPortNoRecord[1].ToString();
		cBox_CameraList->Text = ComPortNoRecord[2].ToString();
	}
			bool f_PlayPause = true;
			int timerInterval = 50;
	private: System::Void Btn_PlayPause_Click(System::Object^  sender, System::EventArgs^  e) {
		if (!f_PlayPause)
		{

			timer2->Enabled = false;
			timer2->Stop();
			Btn_PlayPause->Image = gcnew Bitmap("..\\icon\\arrows.png");
			f_PlayPause = true;
			return;
		}

		if (f_PlayPause)
		{
			timer2->Interval = timerInterval;
			timer2->Enabled = true;
			timer2->Start();

			Btn_PlayPause->Image = gcnew Bitmap("..\\icon\\signs.png");
			f_PlayPause = false;
			return;
		}

	}

	private:double get_Angle(Pt P1, Pt P2)
	{
		return Math::Atan2(P1.y - P2.y, P1.x - P2.x) * 180 / M_PI;
	}
	private:double WipeOutThreshold = 10000, t3 = 0;
	private:bool is_Wipeout(Pt refPt, vector<Pt>&clustPt)
	{
		double sum = 0;
		for (uint i = 0; i < clustPt.size(); i++)
			sum += get_Distance(refPt, clustPt[i]);
		if (sum > WipeOutThreshold)
			return true;
		else
			return false;
	}
	private:uint ReadframeIndex = 0;
	private: System::Void timer2_Tick(System::Object^  sender, System::EventArgs^  e) {
		Mat frame;
		if (cap.isOpened())
		{
			double currentPos = cap.get(CV_CAP_PROP_POS_FRAMES);

			cap.set(CV_CAP_PROP_POS_FRAMES, ReadframeIndex);
			cap >> frame;
			if (!frame.empty()) {
				ShowImage(pictureBox2, frame);
			}
			cout << currentPos << endl;
		}
		vector<Pt>Pt_newClusterRefPt;
		char line[10000];
		vector<vector<Pt>> Pt_ClusterList_new;
		if (fp_LiDarReader.getline(line, sizeof(line), '\n'))
		{
			chart2->Series["Series_LiDAR"]->Points->Clear();
			chart2->Series[1]->Points->Clear();
#pragma region 讀取光達點
			System::String^ str = gcnew System::String(line);
			cli::array<System::String^> ^StringArray = str->Split(' ');
			vector<Pt>LIDAR_cooridate;
			if (StringArray->Length > 10)
			{
				for (uint i = 0; i < 361; i++)
				{
					LIDAR_X_cooridate[i] = System::Convert::ToDouble(StringArray[i]) * cos((0.5 * i) * (M_PI / 180));
					LIDAR_Y_cooridate[i] = System::Convert::ToDouble(StringArray[i]) * sin((0.5 * i) * (M_PI / 180));
					if (System::Convert::ToDouble(StringArray[i]) < 4000 && abs(LIDAR_X_cooridate[i]) < 1000 && TBox.L_RADAR_Mode != 2)
						LIDAR_cooridate.push_back(Pt(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i], System::Convert::ToDouble(StringArray[i]), 0.5 * i));
					else if (System::Convert::ToDouble(StringArray[i]) < 4000 && abs(LIDAR_X_cooridate[i]) < 2500 && TBox.L_RADAR_Mode == 2)
						LIDAR_cooridate.push_back(Pt(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i], System::Convert::ToDouble(StringArray[i]), 0.5 * i));

				}
				CurrentSpeed = System::Convert::ToDouble(StringArray[362]);
				if (Pt_oldClusterRefPoint.size() == 0) {
					Pt_oldClusterRefPoint = Pt_newClusterRefPt;
					t3 = System::Convert::ToDouble(StringArray[361]);
				}
				float t4 = System::Convert::ToDouble(StringArray[361]);
#pragma endregion

				
				int nObj = EuclidCluster(LIDAR_cooridate, 200);
				//int nObj = DBSCAN(LIDAR_cooridate, 100, 2);
				Pt_ClusterList_new = Cluster2List(LIDAR_cooridate, nObj);
				//vector<vector<Pt>> Pt_ClusterList_new = Cluster2List_ContinuousAngle(LIDAR_cooridate);
				//Pt_newClusterRefPt.resize(Pt_ClusterList_new.size());
				int index = 0;
				for (uint16_t i = 0; i < Pt_ClusterList_new.size(); i++)
				{

					double  minX = 8000;
					double minY = 8000;
					Pt min;
					Color color = Color::FromArgb(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
					for (uint j = 0; j < Pt_ClusterList_new[i].size(); j++)
					{
						if (cBox_ScanPt->Checked)
						{
							chart2->Series["Series_LiDAR"]->Points->AddXY(Pt_ClusterList_new[i][j].x, Pt_ClusterList_new[i][j].y);
							chart2->Series["Series_LiDAR"]->Points[index]->Color = color;
						}

						index++;
					}
					if (Pt_ClusterList_new[i].size() > 4)
					{
						if (get_Distance(Pt_ClusterList_new[i][2], Pt_ClusterList_new[i][1]) < 60 && get_Distance(Pt_ClusterList_new[i][2], Pt_ClusterList_new[i][3]) < 60)
							Pt_newClusterRefPt.push_back(Pt_ClusterList_new[i][2]);
						else
							Pt_newClusterRefPt.push_back(Pt_ClusterList_new[i][1]);
					}
					else
					{
						Pt_ClusterList_new.erase(Pt_ClusterList_new.begin() + i);
						i--;
					}

					//Pt_newClusterRefPt[i].range = get_Distance(min, Pt(0, 0));
				}

				float time = (float)(t4 - t3) / CLK_TCK;
				t3 = t4;

				FindClosePoint(Pt_newClusterRefPt, Pt_oldClusterRefPoint, time, CurrentSpeed);
				for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
				{
					chart2->Series[1]->Points->AddXY(Pt_newClusterRefPt[i].x, Pt_newClusterRefPt[i].y);
					if (abs(Pt_newClusterRefPt[i].velcity - CurrentSpeed) > 1 && cBox_ShowLiText->Checked)
						if (_is_InRange(Pt_newClusterRefPt[i]))
							chart2->Series[1]->Points[i]->Label = "(" + Math::Round(Pt_newClusterRefPt[i].x).ToString() + " , " + Math::Round(Pt_newClusterRefPt[i].y).ToString() + " , " + Math::Round(Pt_newClusterRefPt[i].velcity).ToString() + ")";
				}
				Tx_CarSpeed2->Text = CurrentSpeed.ToString();
				Pt_oldClusterRefPoint = Pt_newClusterRefPt;
			}
		}
		else
		{
			fp_LiDarReader.close();
		}
#pragma region Radar
		if (fp_TBoxReader.getline(line, sizeof(line), '\n'))
		{
			chart2->Series["Series_TBox_Radar"]->Points->Clear();
			System::String^ str = gcnew System::String(line);
			cli::array<System::String^> ^StringArray = str->Split(' ');
			TBox.currentSpeed = System::Convert::ToInt32(StringArray[0]);
			TBox.L_RADAR_Mode = System::Convert::ToInt32(StringArray[1]);
			TBox.Driver_RADAR_ALert = System::Convert::ToInt32(StringArray[2]);
			TBox.L_RADAR_Range = System::Convert::ToInt32(StringArray[3]);
			TBox.L_RADAR_Speed = System::Convert::ToInt32(StringArray[4]);
			TBox.L_RADAR_Angle = System::Convert::ToInt32(StringArray[5]);
			TBox.R_RADAR_Mode = System::Convert::ToInt32(StringArray[6]);
			TBox.Passenger_RADAR_ALert = System::Convert::ToInt32(StringArray[7]);
			TBox.R_RADAR_Range = System::Convert::ToInt32(StringArray[8]);
			TBox.R_RADAR_Speed = System::Convert::ToInt32(StringArray[9]);
			TBox.R_RADAR_Angle = System::Convert::ToInt32(StringArray[10]);


			if (std::abs(f_model_changed - TBox.L_RADAR_Mode) > 0)
			{
				//
				f_model_changed = TBox.L_RADAR_Mode;
				chart2->Series["Series_RadarDetectArea"]->Points->Clear();
				DrawBoundary(chart2, TBox.L_RADAR_Mode);
				Tx_TBox_mode->Text = "L:" + getRadarMode(TBox.L_RADAR_Mode) + "  R:" + getRadarMode(TBox.R_RADAR_Mode);
			}

			if (TBox.Passenger_RADAR_ALert)
			{
				tuple<Pt, Pt,Mat> temp;
				Pt R_RadarPtAtLiDAR = R_Radar2LiDAR(Pt(100 * TBox.R_RADAR_Range*Math::Cos(TBox.R_RADAR_Angle*M_PI / 180.f), 100 * TBox.R_RADAR_Range*Math::Sin(TBox.R_RADAR_Angle*M_PI / 180.f)));
				R_RadarPtAtLiDAR.range = TBox.R_RADAR_Range * 100;
				R_RadarPtAtLiDAR.theta = TBox.R_RADAR_Angle;
				R_RadarPtAtLiDAR.velcity = TBox.R_RADAR_Speed;
				//temp.first = R_RadarPtAtLiDAR;

				tx_TBox_RDataP3->ForeColor = Color::Red;
				tx_TBox_RDataP3->Text = "R Range: " + TBox.R_RADAR_Range.ToString() + " R Angle: " + TBox.R_RADAR_Angle.ToString() + "R Speed: " + TBox.R_RADAR_Speed.ToString();
				double minDistant = 8000000;
				double distant;
				uint closePtIndex = 0;
				for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
				{
					distant = get_Distance(Pt_newClusterRefPt[i], R_RadarPtAtLiDAR);

					if (distant < minDistant)
					{
						minDistant = distant;
						closePtIndex = i;
					}
				}
				Pt minToLidar;
				double distant_in = 8000;
				for (uint i = 0; i < Pt_ClusterList_new[closePtIndex].size(); i++)
				{
					if (distant_in > get_Distance(Pt_ClusterList_new[closePtIndex][i], R_RadarPtAtLiDAR))
					{
						distant_in = get_Distance(Pt_ClusterList_new[closePtIndex][i], R_RadarPtAtLiDAR);
						minToLidar = Pt_ClusterList_new[closePtIndex][i];
						minToLidar.velcity = Pt_newClusterRefPt[closePtIndex].velcity;
					}
				}
				temp =make_tuple(R_RadarPtAtLiDAR, minToLidar,frame);
				StatisticPt.push_back(temp);
				RadarAlertIndex.push_back(ReadframeIndex);
				chart2->Series["Series_TBox_Radar"]->Points->AddXY(R_RadarPtAtLiDAR.x, R_RadarPtAtLiDAR.y);
			}
			else
			{
				tx_TBox_RDataP3->ForeColor = Color::Blue;
			}
			if (TBox.Driver_RADAR_ALert)
			{
				tuple<Pt, Pt,Mat>temp;
				tx_TBox_LDataP3->ForeColor = Color::Red;
				tx_TBox_LDataP3->Text = "L Range: " + TBox.L_RADAR_Range.ToString() + " L Angle: " + TBox.L_RADAR_Angle.ToString() + " L Speed: " + TBox.L_RADAR_Speed.ToString();
				Pt L_RadarPtAtLiDAR = L_Radar2LiDAR(Pt(100 * TBox.L_RADAR_Range*Math::Cos(TBox.L_RADAR_Angle*M_PI / 180.f), 100 * TBox.L_RADAR_Range*Math::Sin(TBox.L_RADAR_Angle*M_PI / 180.f)));
				L_RadarPtAtLiDAR.velcity = TBox.L_RADAR_Speed;
				L_RadarPtAtLiDAR.range = TBox.L_RADAR_Range * 100;
				L_RadarPtAtLiDAR.theta = TBox.L_RADAR_Angle;
				
				double minDistant = 8000000;
				double distant;
				uint closePtIndex = 0;

				for (uint i = 0; i < Pt_newClusterRefPt.size(); i++)
				{
					distant = get_Distance(Pt_newClusterRefPt[i], L_RadarPtAtLiDAR);

					if (distant < minDistant)
					{
						minDistant = distant;
						closePtIndex = i;
					}
				}
				Pt min2Lidar;
				double distant_in = 8000;
				for (uint i = 0; i < Pt_ClusterList_new[closePtIndex].size(); i++)
				{
					if (distant_in > get_Distance(Pt_ClusterList_new[closePtIndex][i], L_RadarPtAtLiDAR))
					{
						distant_in = get_Distance(Pt_ClusterList_new[closePtIndex][i], L_RadarPtAtLiDAR);
						min2Lidar = Pt_ClusterList_new[closePtIndex][i];
						min2Lidar.velcity = Pt_newClusterRefPt[closePtIndex].velcity;

					}
				}
				temp=make_tuple(L_RadarPtAtLiDAR,min2Lidar,frame);
				RadarAlertIndex.push_back(ReadframeIndex);
				StatisticPt.push_back(temp);
				chart2->Series["Series_TBox_Radar"]->Points->AddXY(L_RadarPtAtLiDAR.x, L_RadarPtAtLiDAR.y);
			}

			else
				tx_TBox_LDataP3->ForeColor = Color::Blue;
		}
#pragma endregion
		if (ReadframeIndex <cap.get(CV_CAP_PROP_FRAME_COUNT))
		{
			trackBar1->Value = ReadframeIndex;
			Tx_BarPos->Text = ReadframeIndex.ToString();
			ReadframeIndex++;
		}
		else
		{
	        Btn_PlayPause->PerformClick();
			MessageBox::Show("End of File");	
			cBox_ShowResult->Visible = true;
			numericUpDown1->Visible = true;
			numericUpDown1->Maximum = StatisticPt.size()-1;
		}
	}
	private: System::Void Btn_UpDateFileName_Click(System::Object^  sender, System::EventArgs^  e) {

		char timeNow[30] = { 0 };
		uint currnetTime = System::DateTime::Now.Hour * 10000 + System::DateTime::Now.Minute * 100 + System::DateTime::Now.Second;
		sprintf(timeNow, "%d", currnetTime);
		FileNameTime = (string)"RecordData" + (string)timeNow;
		std::string str = (string)"mkdir " + FileNameTime;
		system(str.c_str());
		str = FileNameTime + (string)"\\VideoTest.avi";
		videoWrite.open(str, CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640, 480));
	}
	private: System::Void Btn_LoadFilePath_Click(System::Object^  sender, System::EventArgs^  e) {
		FolderBrowserDialog^ openFileDialog1 = gcnew FolderBrowserDialog;

		if (openFileDialog1->ShowDialog() == System::Windows::Forms::DialogResult::OK)
		{
			LoadFilePath = (char*)(void*)Marshal::StringToHGlobalAnsi(openFileDialog1->SelectedPath);
			tBox_LoadPath->Text = openFileDialog1->SelectedPath;
			if (fp_LiDarReader.is_open())fp_LiDarReader.close();
			fp_LiDarReader.open(LoadFilePath + "\\Lidar.txt", ios::in);
			if (cap.isOpened())cap.release();
			cap.open(LoadFilePath + "\\VideoTest.avi");
			if (fp_TBoxReader.is_open())fp_TBoxReader.close();
			fp_TBoxReader.open(LoadFilePath + "\\TBox.txt", ios::in);
			Tx_MaxFrame->Text = cap.get(CV_CAP_PROP_FRAME_COUNT).ToString();
			trackBar1->Maximum = cap.get(CV_CAP_PROP_FRAME_COUNT) + 1;
			Btn_PlayPause->Enabled = true;
			ReadframeIndex = 0;
			cBox_ShowResult->Visible = false;
			StatisticPt.resize(0);
			RadarAlertIndex.resize(0);
		}

	}
	private: System::Void trackBar1_Scroll(System::Object^  sender, System::EventArgs^  e) {
		Btn_PlayPause->PerformClick();
		Tx_BarPos->Text = trackBar1->Value.ToString();
		ReadframeIndex = trackBar1->Value;
		fp_LiDarReader.close();
		fp_TBoxReader.close();
		cap.release();
		fp_LiDarReader.open(LoadFilePath + "\\Lidar.txt", ios::in);
		cap.open(LoadFilePath + "\\VideoTest.avi");
		fp_TBoxReader.open(LoadFilePath + "\\TBox.txt", ios::in);
		char line[10000] = { 0 };
		for (uint i = 0; i < ReadframeIndex; i++)
		{
			fp_TBoxReader.getline(line, sizeof(line));
			fp_LiDarReader.getline(line, sizeof(line));
		}
		cap.set(CV_CAP_PROP_POS_FRAMES, ReadframeIndex);
		Pt_oldClusterRefPoint.resize(0);
		Btn_PlayPause->PerformClick();

	}
	private: System::Void Btn_SpeedUp_Click(System::Object^  sender, System::EventArgs^  e) {
		timerInterval -= 5;
		if (timerInterval > 1)
			timer2->Interval = timerInterval;
	}
	private: System::Void Btn_SlowDown_Click(System::Object^  sender, System::EventArgs^  e) {

		timerInterval += 5;
		timer2->Interval = timerInterval;
	}

	private: System::Void cBox_ShowResult_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		if (cBox_ShowResult->Checked)
		{
			for (uint i = 0; i < 3; i++)
				chart2->Series[i]->Points->Clear();
			for (uint i = 0; i < StatisticPt.size(); i++)
			{
				
				chart2->Series[1]->Points->AddXY(std::get<1>(StatisticPt[i]).x, std::get<1>(StatisticPt[i]).y);
				chart2->Series[2]->Points->AddXY(std::get<0>(StatisticPt[i]).x, std::get<0>(StatisticPt[i]).y);
			}
		}
		else
		{
			for (uint i = 0; i < 3; i++)
				chart2->Series[i]->Points->Clear();
		}
	}
	private: System::Void numericUpDown1_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
		for (uint i = 0; i < 3; i++)
			chart2->Series[i]->Points->Clear();
		
		uint index= Convert::ToInt32(numericUpDown1->Value);
		Tx_BarPos->Text=index.ToString();
		chart2->Series[1]->Points->AddXY(std::get<1>(StatisticPt[index]).x, std::get<1>(StatisticPt[index]).y);
		chart2->Series[2]->Points->AddXY(std::get<0>(StatisticPt[index]).x, std::get<0>(StatisticPt[index]).y);
	
		Tx_Result_LIDAR->Text = "LIDAR: X:"+ Math::Round(std::get<1>(StatisticPt[index]).x).ToString()+" Y: "+ Math::Round(std::get<1>(StatisticPt[index]).y).ToString()+" V:"+ Math::Round(std::get<1>(StatisticPt[index]).velcity).ToString()+" Index:"+ RadarAlertIndex[index].ToString();
		Tx_Result_Radar->Text= "Radar: X:" + Math::Round(std::get<0>(StatisticPt[index]).x).ToString() + " Y: " + Math::Round(std::get<0>(StatisticPt[index]).y).ToString() + " V:" + Math::Round(std::get<0>(StatisticPt[index]).velcity).ToString();
		ShowImage(pictureBox2,std::get<2>(StatisticPt[index]));
		trackBar1->Value = RadarAlertIndex[index];
		if(fp_LiDarReader.is_open())fp_LiDarReader.close();
		fp_LiDarReader.open(LoadFilePath + "\\Lidar.txt", ios::in);
		char line[10000] = { 0 };
		for (uint i = 0; i < RadarAlertIndex[index]+1; i++)
		{
			fp_LiDarReader.getline(line, sizeof(line));
		}
		System::String^ str = gcnew System::String(line);
		cli::array<System::String^> ^StringArray = str->Split(' ');
		for (uint i = 0; i < 361; i++)
		{
		chart2->Series[0]->Points->AddXY(System::Convert::ToDouble(StringArray[i]) * cos((0.5 * i) * (M_PI / 180)), System::Convert::ToDouble(StringArray[i]) * sin((0.5 * i) * (M_PI / 180)));
		}
		fp_LiDarReader.close();
	}
};
}
