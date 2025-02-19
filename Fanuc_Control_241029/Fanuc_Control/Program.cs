using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Data.SqlClient;
using Newtonsoft.Json;
using System.IO;
using System.Threading;


namespace Fanuc_Control
{
    class Program
    {
        //private FRRJIf.Core mobjCore;
        //private FRRJIf.DataTable mobjDataTable;
        //private FRRJIf.DataTable mobjDataTable2;

        
        private SqlConnection connection;
        //private SqlCommand command;
        //private SqlDataReader reader;

        static void Main(string[] args)
        {
            bool blnRes = false;
            string connectionString = "Server=192.168.2.105;Database=Fanuc;User Id=sa;Password=pass;";
            FRRJIf.Core mobjCore = new FRRJIf.Core();//創建核心
            FRRJIf.DataTable mobjDataTable;//宣告DataTabl
            FRRJIf.DataCurPos mobjCurPos;//宣告座標
            FRRJIf.DataPosReg mobjPosReg;//宣告PR
            mobjDataTable = mobjCore.DataTable;//刷新資料
            mobjCurPos = mobjDataTable.AddCurPos(FRRJIf.FRIF_DATA_TYPE.CURPOS, 1);//從DataTable中取POS
            mobjPosReg = mobjDataTable.AddPosReg(FRRJIf.FRIF_DATA_TYPE.POSREG, 1, 1, 10);//從DataTable中取RR
            //================================================
            string jsonFilePath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "../../Connect_info.json");
            string jsonContent = File.ReadAllText(jsonFilePath);
            dynamic jsonData = JsonConvert.DeserializeObject(jsonContent);
            string RoboHost = jsonData.Robot_IP;
            SqlConnection connection = new SqlConnection(connectionString);
            try
            {
                // 打開資料庫連線
                connection.Open();
                Console.WriteLine("資料庫連線成功！");
            }
            catch (Exception ex)
            {
                Console.WriteLine("資料庫連線失敗：" + ex.Message);
            }
            blnRes = mobjCore.Connect(RoboHost);
            //SetCoord(connection, mobjDataTable, mobjCore);

            //// 測試資料庫連線
            //ReadCoord(connection, mobjCore, mobjDataTable,mobjCurPos);

            // 啟動 SetCoord 和 ReadCoord 兩個方法為非同步任務
            Task setCoordTask = Task.Run(() => SetCoord(connection, mobjDataTable, mobjCore, mobjPosReg, mobjCurPos));
            //Task readCoordTask = Task.Run(() => ReadCoord(connection, mobjCore, mobjDataTable, mobjCurPos));

            // 等待兩個任務完成
            Task.WaitAll(setCoordTask);


            //Console.Read();
        }
        // 測試資料庫連線的方法
        static void ReadCoord(SqlConnection connection, FRRJIf.Core mobjCore, FRRJIf.DataTable mobjDataTable, FRRJIf.DataCurPos mobjCurPos)
        {
            SqlCommand command;

            string J1 = "", J2 = "", J3 = "", J4 = "", J5 = "", J6 = "";
            string X = "", Y = "", Z = "", W = "", P = "", R = "";

            object vntValue = null;
            bool blnDT = false;
            bool blnRes = false;
            bool blnRDO = false;
            Array sngArray = new float[9];
            Array sngJoint = new float[6];
            Array intConfig = new short[7];
            Array xyzwpr = new float[9];
            Array config = new short[7];
            Array joint = new float[9];
            Array intRDO = new short[10];
            short intUF = 0;
            short intUT = 0;
            short intValidC = 0;
            short intValidJ = 0;
            string RDO = "";

            //===================開啟RD1=============================
            Array intVal = new short[10];
            int ii = 0;
            for (ii = 0; ii <= 7; ii++)
            {
                intVal.SetValue((short)0, ii);
            }
            intVal.SetValue((short)1, 0);

            
            while (true)
            {
                blnDT = mobjDataTable.Refresh();
                if (blnDT)
                {
                    blnRDO = mobjCore.ReadRDO(1, ref intRDO, 8);
                    mobjCurPos.GetValue(ref xyzwpr, ref config, ref joint, ref intUF, ref intUT, ref intValidC, ref intValidJ);
   
                    if (intValidJ != 0)//若成功獲得Joint更新當前位置座標
                    {
                        //5
                        for (ii = 0; ii <= 8; ii++)
                        {
                            if (ii == 0) J1 = joint.GetValue(ii).ToString();
                            else if (ii == 1) J2 = joint.GetValue(ii).ToString();
                            else if (ii == 2) J3 = joint.GetValue(ii).ToString();
                            else if (ii == 3) J4 = joint.GetValue(ii).ToString();
                            else if (ii == 4) J5 = joint.GetValue(ii).ToString();
                            else if (ii == 5) J6 = joint.GetValue(ii).ToString();
                        }
                    }
                    if (intValidC != 0)//若成功獲得Coords更新當前位置座標
                    {
                        //5
                        for (ii = 0; ii <= 8; ii++)
                        {
                            if (ii == 0) X = xyzwpr.GetValue(ii).ToString();
                            else if (ii == 1) Y = xyzwpr.GetValue(ii).ToString();
                            else if (ii == 2) Z = xyzwpr.GetValue(ii).ToString();
                            else if (ii == 3) W = xyzwpr.GetValue(ii).ToString();
                            else if (ii == 4) P = xyzwpr.GetValue(ii).ToString();
                            else if (ii == 5) R = xyzwpr.GetValue(ii).ToString();
                        }
                    }
                }


                Console.WriteLine("RD1 : " + intRDO.GetValue(0).ToString());
                Console.WriteLine("RD2 : " + intRDO.GetValue(1).ToString());
                Console.WriteLine("RD3 : " + intRDO.GetValue(2).ToString());
                Console.WriteLine("RD4 : " + intRDO.GetValue(3).ToString());
                Console.WriteLine("RD5 : " + intRDO.GetValue(4).ToString());
                Console.WriteLine("RD6 : " + intRDO.GetValue(5).ToString());
                Console.WriteLine("RD7 : " + intRDO.GetValue(6).ToString());
                Console.WriteLine("RD8 : " + intRDO.GetValue(7).ToString());
                Console.WriteLine("===========================");
                //Console.WriteLine("Y : " + Y);
                //Console.WriteLine("Z : " + Z);
                //Console.WriteLine("W : " + W);
                //Console.WriteLine("P : " + P);
                //Console.WriteLine("R : " + R);
                //Console.WriteLine("J1 : " + J1);
                //Console.WriteLine("J2 : " + J2);
                //Console.WriteLine("J3 : " + J3);
                //Console.WriteLine("J4 : " + J4);
                //Console.WriteLine("J5 : " + J5);
                //Console.WriteLine("J6 : " + J6);

                //// 建立 SQL Server 的連接物件

                try
                {
                    // 打開資料庫連線
                    // 執行查詢操作，例如獲取當前時間
                    //string query = "SELECT * FROM Coord_Status";
                    string query = "UPDATE Coord_Status SET " +
                        "X='" + X + "'," +
                        "Y='" + Y + "'," +
                        "Z='" + Z + "'," +
                        "W='" + W + "'," +
                        "P='" + P + "'," +
                        "R='" + R + "'," +
                        "J1='" + J1 + "'," +
                        "J2='" + J2 + "'," +
                        "J3='" + J3 + "'," +
                        "J4='" + J4 + "'," +
                        "J5='" + J5 + "'," +
                        "J6='" + J6 + "'," +
                        "time = '" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + "';" +
                        "UPDATE IO_Status SET " +
                        "RDO1='" + intRDO.GetValue(0).ToString() + "'," +
                        "RDO2='" + intRDO.GetValue(1).ToString() + "'," +
                        "RDO3='" + intRDO.GetValue(2).ToString() + "'," +
                        "RDO4='" + intRDO.GetValue(3).ToString() + "'," +
                        "RDO5='" + intRDO.GetValue(4).ToString() + "'," +
                        "RDO6='" + intRDO.GetValue(5).ToString() + "'," +
                        "RDO7='" + intRDO.GetValue(6).ToString() + "'," +
                        "RDO8='" + intRDO.GetValue(7).ToString() + "'," +
                        "time = '" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + "' WHERE Source='FANUC';";
                    command = new SqlCommand(query, connection);
                    try
                    {
                        command.ExecuteNonQuery();
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("資料庫連線失敗 - 199：" + ex.Message);
                        command.Dispose();
                    }
                    finally
                    {
                        command.Dispose();
                    }

                }
                catch (Exception ex)
                {
                    Console.WriteLine("資料庫連線失敗 - 207：" + ex.Message);
                    //mobjCore.Disconnect();
                    
                }
                Thread.Sleep(500);
            }

        }
        static void SetCoord(SqlConnection connection, FRRJIf.DataTable mobjDataTable, FRRJIf.Core mobjCore, FRRJIf.DataPosReg mobjPosReg, FRRJIf.DataCurPos mobjCurPos)
        {

            string J1 = "", J2 = "", J3 = "", J4 = "", J5 = "", J6 = "";
            string X = "", Y = "", Z = "", W = "", P = "", R = "",moveType = "";
            int move = 0;
            object vntValue = null;
            bool blnDT = false;
            bool blnRes = false;
            bool blnRDO = false;
            bool UI_control = false;
            bool object_move = false;//偵測後移動
            Array sngArray = new float[9];
            Array sngJoint = new float[6];
            Array intConfig = new short[7];
            Array xyzwpr = new float[9];
            Array config = new short[7];
            Array joint = new float[9];
            short intUF = 0;
            short intUT = 0;
            short intValidC = 0;
            short intValidJ = 0;
            int RD1 = 0, RD2 = 0, RD3 = 0, RD4 = 0, RD5 = 0, RD6 = 0, RD7 = 0, RD8 = 0, RD9 = 0 ;
            int RD1_Temp = 0, RD2_Temp = 0, RD3_Temp = 0, RD4_Temp = 0, RD5_Temp = 0, RD6_Temp = 0, RD7_Temp = 0, RD8_Temp = 0;
            int ii = 0;
            string ONGRAP = "0";
            Array intRDO = new short[10];
            Array intRDOVal = new short[10];

            SqlDataReader reader;
            SqlCommand command;

            while (true)
            {
                //// 建立 SQL Server 的連接物件
                string query = "SELECT * FROM IO_Status;SELECT * FROM PR_Status ORDER BY PR_No ASC";//WHERE PR_No = 'PR[4]';";
                try
                {
                    command = new SqlCommand(query, connection);
                    reader = command.ExecuteReader();
                    try
                    {       
                        // 打開資料庫連線
                        // 執行查詢操作，例如獲取當前時間
                        //string query = "SELECT * FROM Coord_Status";

                        while (reader.Read())
                        {
                            if (reader["Source"].ToString() == "Xavier")
                            {
                                RD1 = Convert.ToInt32(reader["RDO1"].ToString());
                                RD2 = Convert.ToInt32(reader["RDO2"].ToString());
                                RD3 = Convert.ToInt32(reader["RDO3"].ToString());
                                RD4 = Convert.ToInt32(reader["RDO4"].ToString());
                                RD5 = Convert.ToInt32(reader["RDO5"].ToString());
                                RD6 = Convert.ToInt32(reader["RDO6"].ToString());
                                RD7 = Convert.ToInt32(reader["RDO7"].ToString());
                                RD8 = Convert.ToInt32(reader["RDO8"].ToString());
                                RD9 = Convert.ToInt32(reader["RDO9"].ToString());
                                
                            }
                            else
                            {
                                RD1_Temp = Convert.ToInt32(reader["RDO1"].ToString());
                                RD2_Temp = Convert.ToInt32(reader["RDO2"].ToString());
                                RD3_Temp = Convert.ToInt32(reader["RDO3"].ToString());
                                RD4_Temp = Convert.ToInt32(reader["RDO4"].ToString());
                                RD5_Temp = Convert.ToInt32(reader["RDO5"].ToString());
                                RD6_Temp = Convert.ToInt32(reader["RDO6"].ToString());
                                RD7_Temp = Convert.ToInt32(reader["RDO7"].ToString());
                                RD8_Temp = Convert.ToInt32(reader["RDO8"].ToString());
                            }

                        }
                        
                        ///////////////////////////////////////////////////////////////////////////////////////////


                        ///////////////////////////////////////////////////////////////////////////////////////////

                        reader.NextResult();
                        while (reader.Read())
                        {
                            if (reader["PR_No"].ToString() == "PR[3]")
                            {
                                X = reader["X_J1"].ToString();
                                Y = reader["Y_J2"].ToString();
                                Z = reader["Z_J3"].ToString();
                                W = reader["W_J4"].ToString();
                                P = reader["P_J5"].ToString();
                                R = reader["R_J6"].ToString();
                                move = int.Parse(reader["move"].ToString());
                                moveType = reader["moveType"].ToString();
                                ONGRAP = reader["ON_GRAP"].ToString();
                                UI_control = false;
                            }
                            else if (reader["PR_No"].ToString() == "PR[4]")
                            {
                                if (move != 1) // 目標偵測優於
                                {
                                    X = reader["X_J1"].ToString();
                                    Y = reader["Y_J2"].ToString();
                                    Z = reader["Z_J3"].ToString();
                                    W = reader["W_J4"].ToString();
                                    P = reader["P_J5"].ToString();
                                    R = reader["R_J6"].ToString();
                                    move = int.Parse(reader["move"].ToString());
                                    moveType = reader["moveType"].ToString();
                                    UI_control = true;
                                }
                            }
                        }
                        reader.Close();

                        if (move == 1)
                        {
                            if (UI_control == true)
                            {
                                if (moveType == "joint")
                                {
                                    Console.WriteLine("關節");
                                    sngJoint.SetValue((float)Convert.ToDouble(X), 0);
                                    sngJoint.SetValue((float)Convert.ToDouble(Y), 1);
                                    sngJoint.SetValue((float)Convert.ToDouble(Z), 2);
                                    sngJoint.SetValue((float)Convert.ToDouble(W), 3);
                                    sngJoint.SetValue((float)Convert.ToDouble(P), 4);
                                    sngJoint.SetValue((float)Convert.ToDouble(R), 5);
                                    intConfig.SetValue((short)1, 0);//
                                    intConfig.SetValue((short)1, 2);
                                    intConfig.SetValue((short)1, 3);
                                    //mobjPosReg.SetValueXyzwpr(3, ref sngArray, ref intConfig, -1, -1);
                                    mobjPosReg.SetValueJoint(4, ref sngJoint, 15, 15);
                                }
                                else
                                {
                                    sngArray.SetValue((float)Convert.ToDouble(X), 0);
                                    sngArray.SetValue((float)Convert.ToDouble(Y), 1);
                                    sngArray.SetValue((float)Convert.ToDouble(Z), 2);
                                    sngArray.SetValue((float)Convert.ToDouble(W), 3);
                                    sngArray.SetValue((float)Convert.ToDouble(P), 4);
                                    sngArray.SetValue((float)Convert.ToDouble(R), 5);
                                    sngArray.SetValue((float)(0), 6);
                                    sngArray.SetValue((float)(0), 7);
                                    sngArray.SetValue((float)(0), 8);
                                    intConfig.SetValue((short)1, 0);//
                                    intConfig.SetValue((short)1, 2);
                                    intConfig.SetValue((short)1, 3);
                                    mobjPosReg.SetValueXyzwpr(4, ref sngArray, ref intConfig, -1, -1);
                                }
                                intRDOVal = new short[10];
                                for (ii = 0; ii <= 7; ii++)
                                {
                                    intRDOVal.SetValue((short)0, ii);
                                }
                                blnRDO = mobjCore.ReadRDO(1, ref intRDO, 8);
                                intRDOVal.SetValue((short)1, 0);//開啟RD1 夾爪閉合，無自動偵測
                                intRDOVal.SetValue((short)1, 1);//開啟RD2 開啟手動移動模式
                                intRDOVal.SetValue((short)intRDO.GetValue(2), 2);//RD3依據讀取結果回饋
                                blnRes = mobjCore.WriteRDO(1, ref intRDOVal, 8);
                                //================================================
                                query = "UPDATE PR_Status SET move = '0' WHERE PR_No = 'PR[4]';";
                                command = new SqlCommand(query, connection);
                                command.ExecuteNonQuery();
                            }
                            else//視覺目標偵測移動
                            {
                                blnDT = mobjDataTable.Refresh();
                                sngArray.SetValue((float)Convert.ToDouble(X), 0);
                                sngArray.SetValue((float)Convert.ToDouble(Y), 1);
                                sngArray.SetValue((float)Convert.ToDouble(Z), 2);
                                sngArray.SetValue((float)Convert.ToDouble(W), 3);
                                sngArray.SetValue((float)Convert.ToDouble(P), 4);
                                sngArray.SetValue((float)Convert.ToDouble(R), 5);
                                sngArray.SetValue((float)(0), 6);
                                sngArray.SetValue((float)(0), 7);
                                sngArray.SetValue((float)(0), 8); ;
                                intConfig.SetValue((short)1, 0);//F
                                intConfig.SetValue((short)1, 2);//U
                                intConfig.SetValue((short)1, 3);//T
                                                                //設定PR[3]座標位置
                                mobjPosReg.SetValueXyzwpr(3, ref sngArray, ref intConfig, -1, -1);
                                // 先下座標更新PR[3]後再開啟RD1執行移動
                                // =================== 開啟RD1 =============================
                                intRDOVal = new short[10];
                                for (ii = 0; ii <= 7; ii++)
                                {
                                    intRDOVal.SetValue((short)0, ii);
                                }
                                blnRDO = mobjCore.ReadRDO(1, ref intRDO, 8);
                                intRDOVal.SetValue((short)0, 0);//RD1設為0表示夾爪開爪，啟動移動抓取
                                intRDOVal.SetValue((short)0, 1);//RD2設為0表示當前不是手動模式
                                intRDOVal.SetValue((short)intRDO.GetValue(2), 2);//RD3依據讀取結果回饋
                                intRDOVal.SetValue((short)Convert.ToInt32(ONGRAP), 3);//RD4依據讀取結果回饋
                                intRDOVal.SetValue((short)intRDO.GetValue(4), 4);//RD5依據讀取結果回饋
                                blnRes = mobjCore.WriteRDO(1, ref intRDOVal, 8);
                                query = "UPDATE PR_Status SET move = '0' WHERE PR_No = 'PR[3]';";
                                command = new SqlCommand(query, connection);
                                command.ExecuteNonQuery();
                                command.Dispose();
                            }
                        }

                        if (RD9 == 1)
                        {
                            blnDT = mobjDataTable.Refresh();
                            blnRDO = mobjCore.ReadRDO(1, ref intRDO, 8);
                            for (ii = 0; ii <= 7; ii++)
                            {
                                intRDOVal.SetValue((short)0, ii);
                            }
                            intRDOVal.SetValue((short)intRDO.GetValue(0), 0);//RD1設為0表示夾爪開爪，啟動移動抓取
                            intRDOVal.SetValue((short)intRDO.GetValue(1), 1);//RD2設為0表示當前不是手動模式
                            intRDOVal.SetValue((short)intRDO.GetValue(2), 2);//RD3依據讀取結果回饋
                            intRDOVal.SetValue((short)intRDO.GetValue(3), 3);//RD4依據讀取結果回饋
                            intRDOVal.SetValue((short)RD5, 4);//RD5依據讀取結果回饋
                            intRDOVal.SetValue((short)RD6, 5);//RD6依據讀取結果回饋
                            intRDOVal.SetValue((short)RD7, 6);//RD7依據讀取結果回饋
                            intRDOVal.SetValue((short)intRDO.GetValue(7), 7);//RD8依據讀取結果回饋
                            blnRes = mobjCore.WriteRDO(1, ref intRDOVal, 8);
                            query = "UPDATE IO_Status SET RDO9 = '0' WHERE Source = 'Xavier';";
                            command = new SqlCommand(query, connection);
                            command.ExecuteNonQuery();
                            command.Dispose();
                        }                      
                    }
                    catch (Exception ex)
                    {

                        Console.WriteLine("資料庫連線失敗 - 467 ：" + ex.Message);
                        //mobjCore.Disconnect();
                        reader.Close();

                    }

                    blnDT = mobjDataTable.Refresh();
                    if (blnDT)
                    {
                        blnRDO = mobjCore.ReadRDO(1, ref intRDO, 8);
                        mobjCurPos.GetValue(ref xyzwpr, ref config, ref joint, ref intUF, ref intUT, ref intValidC, ref intValidJ);

                        if (intValidJ != 0)//若成功獲得Joint更新當前位置座標
                        {
                            //5
                            for (ii = 0; ii <= 8; ii++)
                            {
                                if (ii == 0) J1 = joint.GetValue(ii).ToString();
                                else if (ii == 1) J2 = joint.GetValue(ii).ToString();
                                else if (ii == 2) J3 = joint.GetValue(ii).ToString();
                                else if (ii == 3) J4 = joint.GetValue(ii).ToString();
                                else if (ii == 4) J5 = joint.GetValue(ii).ToString();
                                else if (ii == 5) J6 = joint.GetValue(ii).ToString();
                            }
                        }
                        if (intValidC != 0)//若成功獲得Coords更新當前位置座標
                        {
                            //5
                            for (ii = 0; ii <= 8; ii++)
                            {
                                if (ii == 0) X = xyzwpr.GetValue(ii).ToString();
                                else if (ii == 1) Y = xyzwpr.GetValue(ii).ToString();
                                else if (ii == 2) Z = xyzwpr.GetValue(ii).ToString();
                                else if (ii == 3) W = xyzwpr.GetValue(ii).ToString();
                                else if (ii == 4) P = xyzwpr.GetValue(ii).ToString();
                                else if (ii == 5) R = xyzwpr.GetValue(ii).ToString();
                            }
                        }
                    }


                    //Console.WriteLine("RD1 : " + intRDO.GetValue(0).ToString());
                    //Console.WriteLine("RD2 : " + intRDO.GetValue(1).ToString());
                    //Console.WriteLine("RD3 : " + intRDO.GetValue(2).ToString());
                    //Console.WriteLine("RD4 : " + intRDO.GetValue(3).ToString());
                    //Console.WriteLine("RD5 : " + intRDO.GetValue(4).ToString());
                    //Console.WriteLine("RD6 : " + intRDO.GetValue(5).ToString());
                    //Console.WriteLine("RD7 : " + intRDO.GetValue(6).ToString());
                    //Console.WriteLine("RD8 : " + intRDO.GetValue(7).ToString());
                    Console.WriteLine("===========================");
                    //Console.WriteLine("Y : " + Y);
                    //Console.WriteLine("Z : " + Z);
                    //Console.WriteLine("W : " + W);
                    //Console.WriteLine("P : " + P);
                    //Console.WriteLine("R : " + R);
                    //Console.WriteLine("J1 : " + J1);
                    //Console.WriteLine("J2 : " + J2);
                    //Console.WriteLine("J3 : " + J3);
                    //Console.WriteLine("J4 : " + J4);
                    ////Console.WriteLine("J5 : " + J5);
                    //Console.WriteLine("J6 : " + J6);

                    //// 建立 SQL Server 的連接物件

                    try
                    {
                        // 打開資料庫連線
                        // 執行查詢操作，例如獲取當前時間
                        //string query = "SELECT * FROM Coord_Status";
                        query = "UPDATE Coord_Status SET " +
                            "X='" + X + "'," +
                            "Y='" + Y + "'," +
                            "Z='" + Z + "'," +
                            "W='" + W + "'," +
                            "P='" + P + "'," +
                            "R='" + R + "'," +
                            "J1='" + J1 + "'," +
                            "J2='" + J2 + "'," +
                            "J3='" + J3 + "'," +
                            "J4='" + J4 + "'," +
                            "J5='" + J5 + "'," +
                            "J6='" + J6 + "'," +
                            "time = '" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + "';" +
                            "UPDATE IO_Status SET " +
                            "RDO1='" + intRDO.GetValue(0).ToString() + "'," +
                            "RDO2='" + intRDO.GetValue(1).ToString() + "'," +
                            "RDO3='" + intRDO.GetValue(2).ToString() + "'," +
                            "RDO4='" + intRDO.GetValue(3).ToString() + "'," +
                            "RDO5='" + intRDO.GetValue(4).ToString() + "'," +
                            "RDO6='" + intRDO.GetValue(5).ToString() + "'," +
                            "RDO7='" + intRDO.GetValue(6).ToString() + "'," +
                            "RDO8='" + intRDO.GetValue(7).ToString() + "'," +
                            "time = '" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + "' WHERE Source='FANUC';";
                        command = new SqlCommand(query, connection);
                        try
                        {
                            command.ExecuteNonQuery();
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine("資料庫連線失敗 - 545：" + ex.Message);
                            command.Dispose();
                        }
                        finally
                        {
                            command.Dispose();
                        }

                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("資料庫連線失敗 - 207：" + ex.Message);
                        //mobjCore.Disconnect();

                    }

                }
                catch
                {
                    Console.WriteLine("資料庫連線失敗 - 475：");
                }
                

                Thread.Sleep(50);
            }
            


        }
    }
}
