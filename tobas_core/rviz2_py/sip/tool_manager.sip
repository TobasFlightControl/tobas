namespace rviz_common
{

class ToolManager: QObject /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz_common"
#include <rviz_common/tool_manager.h>
%End

public:
//  ToolManager( DisplayContext* context );
//  virtual ~ToolManager();

  /** @brief Initialization for after the DisplayContext is created.
   * Loads standard RViz tools. */
  void initialize();

  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;

//  PropertyTreeModel* getPropertyModel() const { return property_tree_model_; }

  /** @brief Create a tool by class lookup name, add it to the list, and return it. */
  rviz_common::Tool* addTool( const QString& tool_class_lookup_name );

  /**
   * \brief Return the tool currently in use.
   * \sa setCurrentTool()
   */
  rviz_common::Tool* getCurrentTool();

  /**
   * \brief Return the tool at a given index in the Tool list.
   * If index is less than 0 or greater than the number of tools, this
   * will fail an assertion.
   */
  rviz_common::Tool* getTool( int index );

  int numTools();
  void removeTool( int index );

  void removeAll();

  /**
   * \brief Set the current tool.
   * The current tool is given all mouse and keyboard events which
   * VisualizationManager receives via handleMouseEvent() and
   * handleChar().
   * \sa getCurrentTool()
   */
  void setCurrentTool( rviz_common::Tool* tool );

  /**
   * \brief Set the default tool.
   *
   * The default tool is selected directly by pressing the Escape key.
   * The default tool is indirectly selected when a Tool returns
   * Finished in the bit field result of Tool::processMouseEvent().
   * This is how control moves from the InitialPoseTool back to
   * MoveCamera when InitialPoseTool receives a left mouse button
   * release event.
   * \sa getDefaultTool()
   */
  void setDefaultTool( rviz_common::Tool* tool );

  /**
   * \brief Get the default tool.
   * \sa setDefaultTool()
   */
  rviz_common::Tool* getDefaultTool();

  QStringList getToolClasses();

//  void handleChar( QKeyEvent* event, RenderPanel* panel );

//  PluginlibFactory<Tool>* getFactory() { return factory_; }

signals:
  /** @brief Emitted when anything changes which will change the saved config file contents. */
  void configChanged();

  /** @brief Emitted by addTool() after the tool is added to the list of tools. */
  void toolAdded( rviz_common::Tool* );

  /** @brief Emitted by setCurrentTool() after the newly chosen tool
   * is activated. */
  void toolChanged( rviz_common::Tool* );

  void toolRemoved( rviz_common::Tool* );

};

};
