#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFormLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QProcess>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <cmath>

namespace sunhorizon_map_annotation_rviz {
class AnnotationPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit AnnotationPanel(QWidget *parent = nullptr) : Panel(parent) {
    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(0,0,0,0);
    auto *scroll = new QScrollArea(this); scroll->setWidgetResizable(true);
    auto *content = new QWidget(scroll); scroll->setWidget(content); outer->addWidget(scroll);
    auto *layout = new QVBoxLayout(content);
    auto *intro = new QLabel("在原始二维预览上标注；候选图仅作对照。\n玻璃：点两端。禁行区：沿边界点选。", this);
    intro->setWordWrap(true); layout->addWidget(intro);
    auto *form = new QFormLayout;
    mode_ = new QComboBox(this); mode_->addItems({"玻璃线（两点）", "禁行多边形", "楼梯多边形"});
    name_ = new QLineEdit("区域-1", this);
    padding_ = new QDoubleSpinBox(this); padding_->setRange(0, 3); padding_->setSingleStep(.05); padding_->setValue(.50); padding_->setSuffix(" 米");
    form->addRow("形状", mode_); form->addRow("名称", name_); form->addRow("外扩距离", padding_);
    layout->addLayout(form);
    button(layout, "开始描点", [this] {
      if (!points_.isEmpty()) { status_->setText("请先完成或取消当前区域"); return; }
      drawing_ = true; drawMode_ = mode_->currentIndex(); selectTool("rviz_default_plugins/PublishPoint"); update();
    });
    button(layout, "撤销一个点", [this] { if (!points_.isEmpty()) points_.removeLast(); update(); });
    button(layout, "完成当前区域", [this] { finish(); });
    button(layout, "取消描点 / 浏览", [this] { points_ = {}; drawing_ = false; selectTool("rviz_default_plugins/MoveCamera"); update(); });
    list_ = new QListWidget(this); list_->setMaximumHeight(115); layout->addWidget(list_);
    button(layout, "删除选中区域", [this] {
      const int index = list_->currentRow(); if (index >= 0) { zones_.removeAt(index); dirty_ = true; checked_->setChecked(false); update(); }
    });
    button(layout, "保存草稿", [this] { save(false); });
    operator_ = new QLineEdit(this); operator_->setPlaceholderText("核对人姓名"); layout->addWidget(operator_);
    checked_ = new QCheckBox("已对照现场核对标注位置", this); layout->addWidget(checked_);
    button(layout, "确认并生成禁行图", [this] { save(true); });
    status_ = new QLabel("正在加载标注会话", this); status_->setWordWrap(true); status_->setTextInteractionFlags(Qt::TextSelectableByMouse); layout->addWidget(status_);
    layout->addStretch(); setEnabled(false);
  }

  void onInitialize() override {
    contextPath_ = qEnvironmentVariable("SUNHORIZON_MAP_ANNOTATION_CONTEXT");
    QFile file(contextPath_);
    if (!file.open(QIODevice::ReadOnly)) { status_->setText("未找到绑定地图的标注会话；请使用地图查看入口启动"); return; }
    context_ = QJsonDocument::fromJson(file.readAll()).object();
    auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abstraction || context_["frame_id"].toString() != "map") return;
    node_ = abstraction->get_raw_node();
    if (qEnvironmentVariable("ROS_DOMAIN_ID") != "179") { status_->setText("标注仅允许在离线查看域中运行"); return; }
    publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/inspection/annotations", rclcpp::QoS(1).transient_local());
    subscription_ = node_->create_subscription<geometry_msgs::msg::PointStamped>("/inspection/clicked_point", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr p) {
        if (p->header.frame_id != "map") return;
        QMetaObject::invokeMethod(this, [this, p] { addPoint(p->point.x, p->point.y); }, Qt::QueuedConnection);
      });
    QFile draft(context_["draft_path"].toString());
    if (draft.open(QIODevice::ReadOnly)) {
      const auto saved = QJsonDocument::fromJson(draft.readAll()).object();
      bool matches = true;
      for (const auto &key : {"map_id", "map_yaml_sha256", "map_image_sha256"}) matches &= saved[key] == context_[key];
      if (!matches) { status_->setText("已有草稿与地图不匹配，未加载；请检查地图版本"); return; }
      zones_ = saved["zones"].toArray();
    }
    setEnabled(true); update();
  }

private:
  template<class F> void button(QVBoxLayout *layout, const QString &text, F function) {
    auto *b = new QPushButton(text, this); layout->addWidget(b); connect(b, &QPushButton::clicked, this, function);
  }
  void selectTool(const QString &id) {
    auto *manager = getDisplayContext()->getToolManager();
    for (int i = 0; i < manager->numTools(); ++i) if (manager->getTool(i)->getClassId() == id) { manager->setCurrentTool(manager->getTool(i)); return; }
  }
  void addPoint(double x, double y) {
    if (!drawing_ || points_.size() >= 200 || (drawMode_ == 0 && points_.size() >= 2)) return;
    const auto origin = context_["origin"].toArray(); const double r = context_["resolution"].toDouble();
    if (!std::isfinite(x) || !std::isfinite(y) || x < origin[0].toDouble() || y < origin[1].toDouble() ||
        x > origin[0].toDouble()+r*context_["width"].toInt() || y > origin[1].toDouble()+r*context_["height"].toInt()) {
      status_->setText("点击位置超出原始二维地图范围"); return;
    }
    points_.append(QJsonArray{x, y}); checked_->setChecked(false); update();
  }
  void finish() {
    if ((drawMode_ == 0 && points_.size() != 2) || (drawMode_ != 0 && points_.size() < 3)) { status_->setText("玻璃需要两个端点，多边形至少三个点"); return; }
    const QString name = name_->text().trimmed();
    if (name.isEmpty()) { status_->setText("请填写区域名称"); return; }
    for (auto zone : zones_) if (zone.toObject()["name"].toString() == name) { status_->setText("区域名称重复"); return; }
    QJsonArray polygon = points_;
    if (drawMode_ == 0) {
      auto a = points_[0].toArray(), b = points_[1].toArray();
      const double dx=b[0].toDouble()-a[0].toDouble(), dy=b[1].toDouble()-a[1].toDouble(), length=std::hypot(dx,dy);
      if (length < .01) { status_->setText("两个端点太近"); return; }
      const double half=context_["resolution"].toDouble()/2, nx=-dy/length*half, ny=dx/length*half;
      polygon={QJsonArray{a[0].toDouble()+nx,a[1].toDouble()+ny}, QJsonArray{b[0].toDouble()+nx,b[1].toDouble()+ny},
               QJsonArray{b[0].toDouble()-nx,b[1].toDouble()-ny}, QJsonArray{a[0].toDouble()-nx,a[1].toDouble()-ny}};
    }
    zones_.append(QJsonObject{{"name",name},{"type",drawMode_==0?"glass":drawMode_==2?"stairs":"keepout"},
      {"inflation_m",padding_->value()},{"polygon_xy_m",polygon}});
    points_={}; drawing_=false; dirty_=true; checked_->setChecked(false);
    name_->setText(QString("区域-%1").arg(zones_.size()+1));
    selectTool("rviz_default_plugins/MoveCamera"); update();
  }
  void update() {
    if (!publisher_) return;
    list_->clear(); for (auto z : zones_) list_->addItem(z.toObject()["name"].toString());
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    Marker clear; clear.action=Marker::DELETEALL; array.markers.push_back(clear);
    int id=0;
    auto line = [&](QJsonArray polygon, bool closed, double width, float alpha) {
      if (polygon.isEmpty()) return;
      if (closed) polygon.append(polygon[0]);
      Marker m; m.header.frame_id="map"; m.header.stamp=node_->now(); m.ns="annotations"; m.id=id++;
      m.type=Marker::LINE_STRIP; m.action=Marker::ADD; m.pose.orientation.w=1;
      m.scale.x=width; m.color.r=1; m.color.g=.45; m.color.a=alpha;
      for (auto p : polygon) { geometry_msgs::msg::Point point; point.x=p.toArray()[0].toDouble(); point.y=p.toArray()[1].toDouble(); point.z=.08; m.points.push_back(point); }
      array.markers.push_back(m);
    };
    for (auto v : zones_) { auto z=v.toObject(); line(z["polygon_xy_m"].toArray(),true,std::max(.04,2*z["inflation_m"].toDouble()),.18); line(z["polygon_xy_m"].toArray(),true,.04,1); }
    line(points_,false,.07,1); publisher_->publish(array);
    status_->setText(QString("%1 个区域；当前 %2 个点。%3\n保存后仍需检查定位和规划，才可试走。")
      .arg(zones_.size()).arg(points_.size()).arg(dirty_?"有未保存修改":"草稿已加载 / 尚无修改"));
  }
  void save(bool accepted) {
    if (!points_.isEmpty()) { status_->setText("请先完成或取消当前描点"); return; }
    if (accepted && (!checked_->isChecked() || operator_->text().trimmed().isEmpty())) { status_->setText("请填写核对人，并勾选现场核对确认"); return; }
    QJsonObject payload;
    for (const auto &key : {"schema_version","frame_id","map_id","map_yaml_sha256","map_image_sha256"}) payload[key]=context_[key];
    payload["zones"]=zones_; payload["accepted_by"]=operator_->text().trimmed(); payload["field_checked"]=checked_->isChecked();
    auto *process=new QProcess(this);
    QStringList args{context_["helper"].toString(),"--context",contextPath_}; if (accepted) args << "--export";
    connect(process, QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished), this,
      [this,process](int code,QProcess::ExitStatus) {
        setEnabled(true);
        if (code) status_->setText("保存失败："+QString::fromUtf8(process->readAllStandardError()));
        else { dirty_=false; const auto result=QJsonDocument::fromJson(process->readAllStandardOutput()).object();
          status_->setText("已保存：\n"+result["saved"].toString()+(result.contains("keepout_yaml")?"\n已生成禁行图，尚未启用导航。":"\n草稿将在下次打开时恢复。")); }
        process->deleteLater();
      });
    connect(process,&QProcess::errorOccurred,this,[this,process](QProcess::ProcessError error) {
      if (error == QProcess::FailedToStart) { setEnabled(true); status_->setText("无法启动保存程序："+process->errorString()); process->deleteLater(); }
    });
    setEnabled(false); status_->setText(accepted?"正在生成禁行图，请稍候……":"正在保存草稿……");
    process->start("/usr/bin/python3",args);
    process->write(QJsonDocument(payload).toJson()); process->closeWriteChannel();
  }
  QComboBox *mode_; QLineEdit *name_, *operator_; QDoubleSpinBox *padding_;
  QListWidget *list_; QCheckBox *checked_; QLabel *status_;
  QJsonObject context_; QJsonArray zones_, points_; QString contextPath_;
  bool drawing_{false}, dirty_{false}; int drawMode_{0};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};
}
PLUGINLIB_EXPORT_CLASS(sunhorizon_map_annotation_rviz::AnnotationPanel, rviz_common::Panel)
#include "panel.moc"
